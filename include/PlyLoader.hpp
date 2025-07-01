#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>

namespace slam3d
{
	class PlyLoader : public rclcpp::Node
	{
	public:
		PlyLoader(const rclcpp::NodeOptions & options, const std::string& name = "ply_loader");

	protected:
		void publishCloud(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
		                        std::shared_ptr<std_srvs::srv::Empty::Response> response);

		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mPclPublisher;
		rclcpp::Service<std_srvs::srv::Empty>::SharedPtr mPublishCloudService;

		sensor_msgs::msg::PointCloud2 mPointCloudMessage;
	};
}
