#include "PointcloudMapperNode.hpp"

#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>


using namespace slam3d;
using namespace std::chrono_literals;

#define TF_TIMEOUT 100ms

PointcloudMapperNode::PointcloudMapperNode() : Node("pointcloud_mapper"), mClock(this->get_clock()),
	mTfBuffer(this->get_clock()), mTfListener(mTfBuffer), mTfBroadcaster(this)
{
	declare_parameter("map_frame", "map");
	declare_parameter("odometry_frame", "odometry");
	declare_parameter("robot_frame", "robot");
	declare_parameter("laser_frame", "laser");

	declare_parameter("scan_resolution", 0.1);
	declare_parameter("map_resolution", 0.1);
	declare_parameter("pose_translation", 0.5);
	declare_parameter("pose_rotation", 0.5);

	declare_parameter("neighbor_radius", 5.0);
	declare_parameter("max_neighbor_links", 1);
	declare_parameter("patch_building_range", 0);
	declare_parameter("min_loop_length", 10);

	mLogger = new Logger(mClock);
	mLogger->setLogLevel(DEBUG);

	mStorage = new MeasurementStorage();
	mGraph = new BoostGraph(mLogger, mStorage);
	mSolver = new G2oSolver(mLogger);
	mPclSensor = new PointCloudSensor("Velodyne", mLogger);

	RegistrationParameters regParams;
	regParams.point_cloud_density = 0.0;
	regParams.maximum_iterations = 10;
	regParams.max_correspondence_distance = 2.0;
	
	mPclSensor->setMinPoseDistance(
		get_parameter("pose_translation").as_double(),
		get_parameter("pose_rotation").as_double());

	mPclSensor->setMapResolution(get_parameter("map_resolution").as_double());
	mPclSensor->setRegistrationParameters(regParams, false);
	mPclSensor->setNeighborRadius(get_parameter("neighbor_radius").as_double(), get_parameter("max_neighbor_links").as_int());
	mPclSensor->setPatchBuildingRange(get_parameter("patch_building_range").as_int());
	mPclSensor->setMinLoopLength(get_parameter("min_loop_length").as_int());
	mPclSensor->setLinkPrevious(true);
	
	mMapFrame = get_parameter("map_frame").as_string();
	mOdometryFrame = get_parameter("odometry_frame").as_string();
	mRobotFrame = get_parameter("robot_frame").as_string();

	mDrift.header.frame_id = mMapFrame;
	mDrift.child_frame_id = mOdometryFrame;
	
	mGraph->setSolver(mSolver);
	
	mMapper = new Mapper(mGraph, mLogger, Transform::Identity());
	mMapper->registerSensor(mPclSensor);
	
	mTfOdom = new TfOdometry(mGraph, mLogger, &mTfBuffer, TF_TIMEOUT, mRobotFrame, mOdometryFrame);
	mMapper->registerPoseSensor(mTfOdom);
	
	mScanSubscriber = create_subscription<sensor_msgs::msg::PointCloud2>("scan", 10,
		std::bind(&PointcloudMapperNode::scanCallback, this, std::placeholders::_1));
	
	mTransformTimer = create_wall_timer(100ms, std::bind(&PointcloudMapperNode::timerCallback, this));
	
	mMapPublisher = create_publisher<sensor_msgs::msg::PointCloud2>("map", 10);
	
	mGenerateMapService = create_service<std_srvs::srv::Empty>("generate_map",
		std::bind(&PointcloudMapperNode::generateMap, this, std::placeholders::_1, std::placeholders::_2));

	mGraphPublisher = new GraphPublisher(this, mGraph);
	mGraphPublisher->addNodeSensor(mPclSensor->getName(), 0,1,0);
	mGraphPublisher->addEdgeSensor(mPclSensor->getName());
}


void PointcloudMapperNode::timerCallback()
{
	mDrift.header.stamp = mClock.ros_now();
	mTfBroadcaster.sendTransform(mDrift);
}

void PointcloudMapperNode::scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
	try
	{
		PointCloud::Ptr pc(new PointCloud);
		pcl::fromROSMsg(*msg, *pc);
		
		Transform laser_pose = tf2::transformToEigen(
			mTfBuffer.lookupTransform(mRobotFrame, msg->header.frame_id, msg->header.stamp, TF_TIMEOUT));

		Transform odometry_pose = tf2::transformToEigen(
			mTfBuffer.lookupTransform(mOdometryFrame, mRobotFrame, msg->header.stamp, TF_TIMEOUT));

		PointCloud::Ptr scan = mPclSensor->downsample(pc, get_parameter("scan_resolution").as_double());
		
		PointCloudMeasurement::Ptr m(new PointCloudMeasurement(scan, "Robot", mPclSensor->getName(), laser_pose));
		
		if(mPclSensor->addMeasurement(m, odometry_pose))
		{
			mPclSensor->linkLastToNeighbors();
			mDrift = tf2::eigenToTransform(orthogonalize(mPclSensor->getCurrentPose() * odometry_pose.inverse()));
			mDrift.header.frame_id = mMapFrame;
			mDrift.child_frame_id = mOdometryFrame;
		}
	}
	catch(std::exception& e)
	{
		mLogger->message(ERROR, e.what());
	}
}

void PointcloudMapperNode::generateMap(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
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
	
	mGraphPublisher->publishNodes(pc2_msg.header.stamp, pc2_msg.header.frame_id);
	mGraphPublisher->publishEdges(mPclSensor->getName(), pc2_msg.header.stamp, pc2_msg.header.frame_id);
}
