#include "PointcloudMapperNode.hpp"

#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>


using namespace slam3d;
using namespace std::chrono_literals;

#define TF_TIMEOUT 500ms

PointcloudMapperNode::PointcloudMapperNode(const std::string& name) : Node(name),
	mClock(this->get_clock()), mTfBuffer(this->get_clock()), mTfListener(mTfBuffer), mTfBroadcaster(this)
{
	declare_parameter("robot_name", "robot");
	declare_parameter("laser_name", "laser");
	declare_parameter("map_frame", "map");
	declare_parameter("odometry_frame", "odometry");
	declare_parameter("robot_frame", "robot");
	declare_parameter("laser_frame", "laser");

	mRobotName = get_parameter("robot_name").as_string();
	mLaserName = get_parameter("laser_name").as_string();
	mMapFrame = get_parameter("map_frame").as_string();
	mOdometryFrame = get_parameter("odometry_frame").as_string();
	mRobotFrame = get_parameter("robot_frame").as_string();

//	mLogger = new RosLogger(mClock, get_logger());
	mLogger = new Logger(mClock);
	mLogger->setLogLevel(DEBUG);

	mStorage = new MeasurementStorage();
	mGraph = new BoostGraph(mLogger, mStorage);
	mSolver = new G2oSolver(mLogger);
	mPclSensor = new RosPclSensor(mLaserName, mLogger, this);

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
	
	mGenerateCloudService = create_service<std_srvs::srv::Empty>("generate_cloud",
		std::bind(&PointcloudMapperNode::generateCloud, this, std::placeholders::_1, std::placeholders::_2));

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

		PointCloud::Ptr scan = mPclSensor->downsampleScan(pc);
		PointCloudMeasurement::Ptr m(new PointCloudMeasurement(scan, mRobotName, mPclSensor->getName(), laser_pose));
		
		if(mPclSensor->addMeasurement(m, odometry_pose))
		{
			mPclSensor->linkLastToNeighbors();
			mDrift = tf2::eigenToTransform(orthogonalize(mPclSensor->getCurrentPose() * odometry_pose.inverse()));
			mDrift.header.frame_id = mMapFrame;
			mDrift.child_frame_id = mOdometryFrame;

			mGraphPublisher->publishNodes(msg->header.stamp, mMapFrame);
			mGraphPublisher->publishEdges(mPclSensor->getName(), msg->header.stamp, mMapFrame);
		}
	}
	catch(std::exception& e)
	{
		mLogger->message(ERROR, e.what());
	}
}

void PointcloudMapperNode::generateCloud(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
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
