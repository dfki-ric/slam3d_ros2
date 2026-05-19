#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <slam3d/sensor/pcl/PointCloudSensor.hpp>

#include "RosClock.hpp"

namespace slam3d
{
	class Logger;
	class Mapper;
	class BoostGraph;
	class MeasurementStorage;
	class G2oSolver;
	class RosPclSensor;
	class TfOdometry;
	class TfGravity;
	class GraphPublisher;
	class OctoMap;
	
	class PointcloudMapper : public rclcpp::Node
	{
	public:
		PointcloudMapper(const rclcpp::NodeOptions & options, const std::string& name = "pointcloud_mapper");

	protected:

		void timerCallback();

		void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

		void generateCloud(
			const std::shared_ptr<std_srvs::srv::Empty::Request> request,
			std::shared_ptr<std_srvs::srv::Empty::Response> response);

		void removeDynamicObjects(
			const std::shared_ptr<std_srvs::srv::Empty::Request> request,
			std::shared_ptr<std_srvs::srv::Empty::Response> response);

		void exportGraph(
			const std::shared_ptr<std_srvs::srv::Empty::Request> request,
			std::shared_ptr<std_srvs::srv::Empty::Response> response);

		virtual void addScanToMap(const PointCloud::ConstPtr scan, const Transform& pose) {};

		Logger* mLogger;
		Mapper* mMapper;
		BoostGraph* mGraph;
		MeasurementStorage* mStorage;
		G2oSolver* mSolver;
		RosPclSensor* mPclSensor;
		RosClock mClock;
		TfOdometry* mTfOdom;
		TfGravity* mTfGrav;
		OctoMap* mOctomap;
		
		GraphPublisher* mGraphPublisher;
		
		std::string mRobotName;
		std::string mLaserName;
		std::string mMapFrame;
		std::string mOdometryFrame;
		std::string mGravityFrame;
		std::string mRobotFrame;
		int mOptimizationRate;
		
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mMapPublisher;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mRemovedPointsPublisher;
		rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr mOctoMapPublisher;
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mScanSubscriber;
		rclcpp::Service<std_srvs::srv::Empty>::SharedPtr mGenerateCloudService;
		rclcpp::Service<std_srvs::srv::Empty>::SharedPtr mRemoveDynamicObjectsService;
		rclcpp::Service<std_srvs::srv::Empty>::SharedPtr mExportGraphService;
		rclcpp::TimerBase::SharedPtr mTransformTimer;
		
		tf2_ros::Buffer mTfBuffer;
		tf2_ros::TransformListener mTfListener;
		tf2_ros::TransformBroadcaster mTfBroadcaster;
		geometry_msgs::msg::TransformStamped mDrift;
		std::mutex mMutex;
		rclcpp::CallbackGroup::SharedPtr mTfCallbackGroup;

		bool mIsOriginInitialized = false;
	};
}
