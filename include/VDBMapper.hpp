#pragma once

#include <vdb_mapping/OccupancyVDBMapping.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "PointcloudMapper.hpp"

namespace slam3d
{
	class VDBMapper : public PointcloudMapper
	{
	public:
		VDBMapper(const rclcpp::NodeOptions& options, const std::string& name = "vdb_mapper");

	private:

		void generateMap(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
		                       std::shared_ptr<std_srvs::srv::Empty::Response> response);

		void sendMap();

		virtual void addScanToMap(const PointCloud::ConstPtr scan, const Transform& pose) override;

		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mVdbMapPublisher;
		rclcpp::Service<std_srvs::srv::Empty>::SharedPtr mGenerateMapService;

		std::shared_ptr<vdb_mapping::OccupancyVDBMapping> mVdbMapping;
		vdb_mapping::Config mVdbConfig;
		int mAdded = 0;
	};
}
