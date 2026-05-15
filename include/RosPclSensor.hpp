#pragma once

#include <slam3d/sensor/pcl/PointCloudSensor.hpp>
#include <rclcpp/node.hpp>

namespace slam3d
{
	RegistrationParameters declareRegistrationParams(rclcpp::Node* node, const std::string& name);
	
	class RosPclSensor : public PointCloudSensor
	{
	public:
		RosPclSensor(const std::string& name, Logger* logger, rclcpp::Node* node);
	};
}
