#include "PointcloudMapperNode.hpp"

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<slam3d::PointcloudMapperNode>());
	rclcpp::shutdown();
	return 0;
}
