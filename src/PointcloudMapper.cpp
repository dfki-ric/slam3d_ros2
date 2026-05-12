#include "PointcloudMapper.hpp"
#include "RosLogger.hpp"

#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/io/ply_io.h>
#include <functional>
#include <memory>
#include <string>
#include <filesystem>
#include <time.h>

#include <rclcpp_components/register_node_macro.hpp>

#include <octomap_msgs/conversions.h>

#include <slam3d/serialization/GraphSerialization.hpp>
#include <slam3d/serialization/MeasurementSerialization.hpp>

using namespace slam3d;
using namespace std::chrono_literals;

#define TF_TIMEOUT 500ms

PointcloudMapper::PointcloudMapper(const rclcpp::NodeOptions & options, const std::string& name)
:	Node(name, options),
	mClock(this->get_clock()), mTfBuffer(this->get_clock()), mTfListener(mTfBuffer), mTfBroadcaster(this)
{
	declare_parameter("robot_name", "robot");
	declare_parameter("laser_name", "laser");
	declare_parameter("map_frame", "map");
	declare_parameter("odometry_frame", "odometry");
	declare_parameter("gravity_frame", "gravity");
	declare_parameter("robot_frame", "robot");
	declare_parameter("use_odometry", true);
	declare_parameter("use_gravity", false);
	declare_parameter("automatic_optimize", false);
	declare_parameter("optimization_rate", 20);
	declare_parameter("use_odometry_origin", false);
	declare_parameter("initial_map", "");
	declare_parameter("import_graph", false);
	declare_parameter("import_directory", "slam3d_export");

	declare_parameter("map_outlier_radius", 0.0);
	declare_parameter("map_outlier_neighbors", 0);
	declare_parameter("map_resolution", 0.0);
	declare_parameter("map_crop_min_x", -std::numeric_limits<double>::infinity());
	declare_parameter("map_crop_min_y", -std::numeric_limits<double>::infinity());
	declare_parameter("map_crop_min_z", -std::numeric_limits<double>::infinity());
	declare_parameter("map_crop_max_x",  std::numeric_limits<double>::infinity());
	declare_parameter("map_crop_max_y",  std::numeric_limits<double>::infinity());
	declare_parameter("map_crop_max_z",  std::numeric_limits<double>::infinity());

	mRobotName = get_parameter("robot_name").as_string();
	mLaserName = get_parameter("laser_name").as_string();
	mMapFrame = get_parameter("map_frame").as_string();
	mOdometryFrame = get_parameter("odometry_frame").as_string();
	mRobotFrame = get_parameter("robot_frame").as_string();
	mGravityFrame = get_parameter("gravity_frame").as_string();
	mOptimizationRate = get_parameter("optimization_rate").as_int();

	OctoMapConfiguration octoMapConfig;
	declare_parameter("octomap_clamping_thres_max", octoMapConfig.clampingThresMax);
	declare_parameter("octomap_clamping_thres_min", octoMapConfig.clampingThresMin);
	declare_parameter("octomap_occupancy_thres", octoMapConfig.occupancyThres);
	declare_parameter("octomap_prob_hit", octoMapConfig.probHit);
	declare_parameter("octomap_prob_miss", octoMapConfig.probMiss);
	declare_parameter("octomap_range_max", octoMapConfig.rangeMax);
	declare_parameter("octomap_resolution", octoMapConfig.resolution);

	octoMapConfig.clampingThresMax = get_parameter("octomap_clamping_thres_max").as_double();
	octoMapConfig.clampingThresMin = get_parameter("octomap_clamping_thres_min").as_double();
	octoMapConfig.occupancyThres = get_parameter("octomap_occupancy_thres").as_double();
	octoMapConfig.probHit = get_parameter("octomap_prob_hit").as_double();
	octoMapConfig.probMiss = get_parameter("octomap_prob_miss").as_double();
	octoMapConfig.rangeMax = get_parameter("octomap_range_max").as_double();
	octoMapConfig.resolution = get_parameter("octomap_resolution").as_double();

	mLogger = new RosLogger(mClock, get_logger());
	mLogger->setLogLevel(DEBUG);

	mStorage = new MeasurementStorage();
	mGraph = new BoostGraph(mLogger, mStorage);
	mSolver = new G2oSolver(mLogger);
	mPclSensor = new RosPclSensor(mLaserName, mLogger, this);
	mPclSensor->setMapOutlierRemoval(
		get_parameter("map_outlier_radius").as_double(),
		get_parameter("map_outlier_neighbors").as_int());
	mPclSensor->setMapResolution(
		get_parameter("map_resolution").as_double());
	mPclSensor->setMapCropBox(
	{
		(float)get_parameter("map_crop_min_x").as_double(),
		(float)get_parameter("map_crop_min_y").as_double(),
		(float)get_parameter("map_crop_min_z").as_double(),
		1.0
	},
	{
		(float)get_parameter("map_crop_max_x").as_double(),
		(float)get_parameter("map_crop_max_y").as_double(),
		(float)get_parameter("map_crop_max_z").as_double(),
		1.0
	});

	mGraph->setSolver(mSolver);

	mMapper = new Mapper(mGraph, mLogger, Transform::Identity());
	mMapper->registerSensor(mPclSensor);
	mMapper->fixFirst();

	if(get_parameter("use_odometry").as_bool())
	{
		mTfOdom = new TfOdometry(mGraph, mLogger, &mTfBuffer, TF_TIMEOUT, mRobotFrame, mOdometryFrame);
		mMapper->registerPoseSensor(mTfOdom);
		mDrift.header.frame_id = mMapFrame;
		mDrift.child_frame_id = mOdometryFrame;
	}else
	{
		mTfOdom = nullptr;
		mDrift.header.frame_id = mMapFrame;
		mDrift.child_frame_id = mRobotFrame;
	}

	if(get_parameter("use_gravity").as_bool())
	{
		mTfGrav = new TfGravity(mGraph, mLogger, &mTfBuffer, TF_TIMEOUT, mRobotFrame, mGravityFrame, Direction::UnitZ());
		mMapper->registerPoseSensor(mTfGrav);
	}else
	{
		mTfGrav = nullptr;
	}

	if(get_parameter("import_graph").as_bool())
	{
		try
		{
			const std::string& dir = get_parameter("import_directory").as_string();
			mLogger->message(INFO, "Importing previous graph.");
			GraphSerialization::fromFile(mGraph, dir + "/graph.yml");
			MeasurementSerialization::fromDirectory(mStorage, dir, true);
		}catch(std::exception& e)
		{
			mLogger->message(FATAL, (boost::format("Importing graph failed: %1%") % e.what()).str());
			rclcpp::shutdown();
		}
	}else
	{
		const std::string path = get_parameter("initial_map").as_string();
		if(!path.empty())
		{
			mLogger->message(INFO, (boost::format("Loading initial map from %1%.") % path).str());
			mPclSensor->loadPLY(path, mRobotName);
		}
	}

	mOctomap = new OctoMap(octoMapConfig, &mClock, mLogger, mGraph);

	mScanSubscriber = create_subscription<sensor_msgs::msg::PointCloud2>("scan", 10,
		std::bind(&PointcloudMapper::scanCallback, this, std::placeholders::_1));
	
	mTfCallbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	mTransformTimer = rclcpp::create_timer(this, this->get_clock(), 100ms, std::bind(&PointcloudMapper::timerCallback, this), mTfCallbackGroup);
	
	mMapPublisher = create_publisher<sensor_msgs::msg::PointCloud2>("map", 10);
	mOctoMapPublisher = create_publisher<octomap_msgs::msg::Octomap>("octomap", 10);
	
	mGenerateCloudService = create_service<std_srvs::srv::Empty>("generate_cloud",
		std::bind(&PointcloudMapper::generateCloud, this, std::placeholders::_1, std::placeholders::_2));

	mRemoveDynamicObjectsService = create_service<std_srvs::srv::Empty>("remove_dynamic_objects",
		std::bind(&PointcloudMapper::removeDynamicObjects, this, std::placeholders::_1, std::placeholders::_2));

	mExportGraphService = create_service<std_srvs::srv::Empty>("export_graph",
		std::bind(&PointcloudMapper::exportGraph, this, std::placeholders::_1, std::placeholders::_2));

	mGraphPublisher = new GraphPublisher(this, mGraph);
	mGraphPublisher->addNodeSensor(mPclSensor->getName(), 0,1,0);
	mGraphPublisher->addEdgeSensor(mPclSensor->getName());
}


void PointcloudMapper::timerCallback()
{
	std::unique_lock<std::mutex> lock(mMutex);
	mDrift.header.stamp = mClock.ros_now();
	mTfBroadcaster.sendTransform(mDrift);
}

void PointcloudMapper::scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
	try
	{
		PointCloud::Ptr pc(new PointCloud);
		pcl::fromROSMsg(*msg, *pc);
		
		Transform laser_pose = tf2::transformToEigen(
			mTfBuffer.lookupTransform(mRobotFrame, msg->header.frame_id, msg->header.stamp, TF_TIMEOUT));

		pcl::Indices indices;
		pcl::removeNaNFromPointCloud(*pc, *pc, indices);
		PointCloud::Ptr scan = mPclSensor->downsampleScan(pc);
		PointCloudMeasurement::Ptr m(new PointCloudMeasurement(scan, mRobotName, mPclSensor->getName(), laser_pose));

		bool added = false;
		if(mTfOdom)
		{
			Transform odometry_pose = tf2::transformToEigen(
				mTfBuffer.lookupTransform(mOdometryFrame, mRobotFrame, msg->header.stamp, TF_TIMEOUT));
			if(!mIsOriginInitialized and get_parameter("use_odometry_origin").as_bool())
			{
				mMapper->setStartPose(odometry_pose);
				mIsOriginInitialized = true;
			}
			added = mPclSensor->addMeasurement(m, odometry_pose);
			{
				std::unique_lock<std::mutex> lock(mMutex);
				mDrift = tf2::eigenToTransform(orthogonalize(mPclSensor->getCurrentPose() * odometry_pose.inverse()));
				mDrift.header.frame_id = mMapFrame;
				mDrift.child_frame_id = mOdometryFrame;
			}
		}else
		{
			added = mPclSensor->addMeasurement(m);
			{
				std::unique_lock<std::mutex> lock(mMutex);
				mDrift = tf2::eigenToTransform(orthogonalize(mPclSensor->getCurrentPose()));
				mDrift.header.frame_id = mMapFrame;
				mDrift.child_frame_id = mRobotFrame;
			}
		}

		if(added)
		{
			mPclSensor->linkLastToNeighbors();
			mGraphPublisher->publishNodes(msg->header.stamp, mMapFrame);
			mGraphPublisher->publishEdges(mPclSensor->getName(), msg->header.stamp, mMapFrame);
			if(get_parameter("automatic_optimize").as_bool() and mGraph->getNumOfNewConstraints() >= mOptimizationRate)
			{
				mGraph->optimize();
			}
		}
	}
	catch(std::exception& e)
	{
		mLogger->message(ERROR, e.what());
	}
}

void PointcloudMapper::generateCloud(
	const std::shared_ptr<std_srvs::srv::Empty::Request> request,
	std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
	mGraph->optimize();
	VertexObjectList vertices = mGraph->getVertices({mPclSensor->getName()});
	PointCloud::Ptr map = mPclSensor->buildMap(vertices);
	sensor_msgs::msg::PointCloud2 pc2_msg;
	pcl::toROSMsg(*map, pc2_msg);
	pc2_msg.header.frame_id = mMapFrame;
	pc2_msg.header.stamp = mClock.ros_now();
	mMapPublisher->publish(pc2_msg);

	std::ostringstream ply_path;
	time_t sec = mClock.now().tv_sec;
	ply_path << "pointcloud-" << std::put_time(std::localtime(&sec), "%Y%m%d-%H%M") << ".ply";
 
	pcl::PLYWriter ply_writer;
	if(ply_writer.write(ply_path.str(), *map) != 0)
	{
		mLogger->message(ERROR, "Failed to write PLY.");
	}
}

void PointcloudMapper::removeDynamicObjects(
	const std::shared_ptr<std_srvs::srv::Empty::Request> request,
	std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
	mGraph->optimize();
	mOctomap->clear();
	for(const VertexObject& v : mGraph->getVerticesByType("slam3d::PointCloudMeasurement"))
	{
		PointCloudMeasurement::Ptr pc = 
			boost::dynamic_pointer_cast<PointCloudMeasurement>(mGraph->getMeasurement(v.index));
		if(pc)
		{
			mOctomap->addMeasurement(pc, v.correctedPose);
		}
	}
	mOctomap->remove_dynamic_objects();
	mOctomap->sendMap();
	
	octomap_msgs::msg::Octomap msg;
	msg.header.stamp = mClock.ros_now();
	msg.header.frame_id = mMapFrame;
	octomap_msgs::binaryMapToMsg(mOctomap->getOcTree(), msg);
	mOctoMapPublisher->publish(msg);
}

void PointcloudMapper::exportGraph(
	const std::shared_ptr<std_srvs::srv::Empty::Request> request,
	std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
	const std::string dir = get_parameter("import_directory").as_string();
	std::filesystem::create_directory(dir);
	GraphSerialization::toFile(mGraph, dir+"/graph.yml");
	MeasurementSerialization::toDirectory(mStorage, dir, true);
}

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(slam3d::PointcloudMapper)
