#include "PointcloudMapperNode.hpp"

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<slam3d::PointcloudMapperNode>("pointcloud_mapper"));
	rclcpp::shutdown();
	return 0;
}
