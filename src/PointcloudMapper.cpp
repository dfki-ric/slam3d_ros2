#include "PointcloudMapper.hpp"
#include "RosLogger.hpp"

#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp_components/register_node_macro.hpp>

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
	declare_parameter("use_odometry_origin", false);

	mRobotName = get_parameter("robot_name").as_string();
	mLaserName = get_parameter("laser_name").as_string();
	mMapFrame = get_parameter("map_frame").as_string();
	mOdometryFrame = get_parameter("odometry_frame").as_string();
	mRobotFrame = get_parameter("robot_frame").as_string();
	mGravityFrame = get_parameter("gravity_frame").as_string();

	mLogger = new RosLogger(mClock, get_logger());
	mLogger->setLogLevel(DEBUG);

	mStorage = new MeasurementStorage();
	mGraph = new BoostGraph(mLogger, mStorage);
	mSolver = new G2oSolver(mLogger);
	mPclSensor = new RosPclSensor(mLaserName, mLogger, this);
	
	mGraph->setSolver(mSolver);
	mGraph->fixNext();
	
	mMapper = new Mapper(mGraph, mLogger, Transform::Identity());
	mMapper->registerSensor(mPclSensor);
	
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

	mScanSubscriber = create_subscription<sensor_msgs::msg::PointCloud2>("scan", 10,
		std::bind(&PointcloudMapper::scanCallback, this, std::placeholders::_1));
	
	mTfCallbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	mTransformTimer = rclcpp::create_timer(this, this->get_clock(), 100ms, std::bind(&PointcloudMapper::timerCallback, this), mTfCallbackGroup);
	
	mMapPublisher = create_publisher<sensor_msgs::msg::PointCloud2>("map", 10);
	
	mGenerateCloudService = create_service<std_srvs::srv::Empty>("generate_cloud",
		std::bind(&PointcloudMapper::generateCloud, this, std::placeholders::_1, std::placeholders::_2));

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
			if(get_parameter("automatic_optimize").as_bool())
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

void PointcloudMapper::generateCloud(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                               std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
	mGraph->optimize();
	VertexObjectList vertices = mGraph->getVerticesFromSensor(mPclSensor->getName());
	PointCloud::Ptr map = mPclSensor->buildMap(vertices);
	sensor_msgs::msg::PointCloud2 pc2_msg;
	pcl::toROSMsg(*map, pc2_msg);
	pc2_msg.header.frame_id = mMapFrame;
	pc2_msg.header.stamp = mClock.ros_now();
	mMapPublisher->publish(pc2_msg);
}

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(slam3d::PointcloudMapper)
