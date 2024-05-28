#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <slam3d/graph/boost/BoostGraph.hpp>
#include <slam3d/solver/g2o/G2oSolver.hpp>
#include <slam3d/sensor/pcl/PointCloudSensor.hpp>
#include <slam3d/core/Mapper.hpp>

#include <RosClock.hpp>
#include <GraphPublisher.hpp>

using namespace std::chrono_literals;
using namespace slam3d;

#define TF_TIMEOUT 50ms

class TfOdometry : public PoseSensor
{
public:
	TfOdometry(Graph* g, Logger* l, tf2_ros::Buffer* tf, const std::string& robot_f, const std::string& odom_f)
	: PoseSensor("Odometry", g, l), mTfBuffer(tf), mRobotFrame(robot_f), mOdometryFrame(odom_f)
	{
		mLastVertex = 0;
	}
	
	Transform getPose(timeval stamp)
	{
		try
		{
			return tf2::transformToEigen(mTfBuffer->lookupTransform(mOdometryFrame, mRobotFrame, fromTimeval(stamp), TF_TIMEOUT));
		}catch (const tf2::TransformException & ex)
		{
			throw InvalidPose(ex.what());
		}
	}
	
	void handleNewVertex(IdType vertex)
	{
		timeval stamp = mGraph->getVertex(vertex).timestamp;
		Transform currentPose = getPose(stamp);
		
		if(mLastVertex > 0)
		{
			Transform t = mLastOdometricPose.inverse() * currentPose;
			SE3Constraint::Ptr se3(new SE3Constraint(mName, t, Covariance<6>::Identity() * 0.01));
			try
			{
				mGraph->addConstraint(mLastVertex, vertex, se3);
			}catch(std::exception &e)
			{
				std::cout << "Failed to link vertex " << vertex << " to " << mLastVertex << std::endl;
				return;
			}
			mGraph->setCorrectedPose(vertex, mGraph->getVertex(mLastVertex).correctedPose * t);
		}
		
		mLastVertex = vertex;
		mLastOdometricPose = currentPose;
	}
	
private:
	tf2_ros::Buffer* mTfBuffer;
	Transform mLastOdometricPose;
	IdType mLastVertex;
	std::string mRobotFrame;
	std::string mOdometryFrame;

};

class PointcloudMapper : public rclcpp::Node
{
public:
	PointcloudMapper() : Node("pointcloud_mapper"), mClock(this->get_clock()), mTfBuffer(this->get_clock()),
		mTfListener(mTfBuffer), mTfBroadcaster(this)
	{
		declare_parameter("map_frame", "map");
		declare_parameter("odometry_frame", "odometry");
		declare_parameter("robot_frame", "robot");
		declare_parameter("laser_frame", "laser");

		declare_parameter("scan_resolution", 0.1);
		declare_parameter("map_resolution", 0.1);
		declare_parameter("pose_translation", 0.5);
		declare_parameter("pose_rotation", 0.5);

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
		mPclSensor->setNeighborRadius(5.0, 1);
		mPclSensor->setLinkPrevious(true);
		
		mMapFrame = get_parameter("map_frame").as_string();
		mOdometryFrame = get_parameter("odometry_frame").as_string();
		mRobotFrame = get_parameter("robot_frame").as_string();

		mDrift.header.frame_id = mMapFrame;
		mDrift.child_frame_id = mOdometryFrame;
		
		mGraph->setSolver(mSolver);
		
		mMapper = new Mapper(mGraph, mLogger, Transform::Identity());
		mMapper->registerSensor(mPclSensor);
		
		mTfOdom = new TfOdometry(mGraph, mLogger, &mTfBuffer, mRobotFrame, mOdometryFrame);
		mMapper->registerPoseSensor(mTfOdom);
		
		mScanSubscriber = create_subscription<sensor_msgs::msg::PointCloud2>("scan", 10,
			std::bind(&PointcloudMapper::scanCallback, this, std::placeholders::_1));
		
		mTransformTimer = create_wall_timer(100ms, std::bind(&PointcloudMapper::timerCallback, this));
		
		mMapPublisher = create_publisher<sensor_msgs::msg::PointCloud2>("map", 10);
		
		mGenerateMapService = create_service<std_srvs::srv::Empty>("generate_map",
			std::bind(&PointcloudMapper::generateMap, this, std::placeholders::_1, std::placeholders::_2));

		mGraphPublisher = new GraphPublisher(this, mGraph);
		mGraphPublisher->addNodeSensor(mPclSensor->getName(), 0,1,0);
		mGraphPublisher->addEdgeSensor(mPclSensor->getName());
	}

private:

	void timerCallback()
	{
		mDrift.header.stamp = mClock.ros_now();
		mTfBroadcaster.sendTransform(mDrift);
	}

	void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
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

	void generateMap(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
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

	Mapper* mMapper;
	BoostGraph* mGraph;
	MeasurementStorage* mStorage;
	G2oSolver* mSolver;
	PointCloudSensor* mPclSensor;
	RosClock mClock;
	Logger* mLogger;
	TfOdometry* mTfOdom;
	
	GraphPublisher* mGraphPublisher;
	
	std::string mMapFrame;
	std::string mOdometryFrame;
	std::string mRobotFrame;
	
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mMapPublisher;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mScanSubscriber;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr mGenerateMapService;
	rclcpp::TimerBase::SharedPtr mTransformTimer;
	
	tf2_ros::Buffer mTfBuffer;
	tf2_ros::TransformListener mTfListener;
	tf2_ros::TransformBroadcaster mTfBroadcaster;
	geometry_msgs::msg::TransformStamped mDrift;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PointcloudMapper>());
	rclcpp::shutdown();
	return 0;
}
