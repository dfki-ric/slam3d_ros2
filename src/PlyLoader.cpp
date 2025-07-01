#include "PlyLoader.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>

using namespace slam3d;

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloud;

PlyLoader::PlyLoader(const rclcpp::NodeOptions & options, const std::string& name)
:	Node(name, options)
{
	declare_parameter("ply", "");
	declare_parameter("frame_id", "ply_cloud");
	mPclPublisher = create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 1);

	mPublishCloudService = create_service<std_srvs::srv::Empty>("publish_cloud",
		std::bind(&PlyLoader::publishCloud, this, std::placeholders::_1, std::placeholders::_2));

	PointCloud pcl_cloud;
	pcl::PLYReader ply_reader;
	if(ply_reader.read(get_parameter("ply").as_string(), pcl_cloud) >= 0)
	{
		pcl::toROSMsg(pcl_cloud, mPointCloudMessage);
		mPointCloudMessage.header.frame_id = get_parameter("frame_id").as_string();
	}else
	{
		RCLCPP_ERROR(get_logger(), "Failed to load PLY!");
	}
}

void PlyLoader::publishCloud(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                   std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
	mPointCloudMessage.header.stamp = get_clock()->now();
	mPclPublisher->publish(mPointCloudMessage);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(slam3d::PlyLoader)
