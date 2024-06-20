#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <slam3d/graph/boost/BoostGraph.hpp>
#include <slam3d/solver/g2o/G2oSolver.hpp>
#include <slam3d/sensor/pcl/PointCloudSensor.hpp>
#include <slam3d/core/Mapper.hpp>

#include "RosClock.hpp"
#include "GraphPublisher.hpp"
#include "TfOdometry.hpp"

namespace slam3d
{
	class PointcloudMapperNode : public rclcpp::Node
	{
	public:
		PointcloudMapperNode(const std::string& name);

	protected:

		void timerCallback();

		void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

		void generateMap(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
							   std::shared_ptr<std_srvs::srv::Empty::Response> response);

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
}
