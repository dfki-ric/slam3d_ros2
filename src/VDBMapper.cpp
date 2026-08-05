#include "VDBMapper.hpp"

#include <vdb_mapping_ros2/VDBMappingTools.hpp>

#include <slam3d/core/Mapper.hpp>
#include <slam3d/graph/boost/BoostGraph.hpp>
#include <slam3d_ros2/RosPclSensor.hpp>

#include <boost/format.hpp>

using namespace slam3d;

VDBMapper::VDBMapper(const rclcpp::NodeOptions & options, const std::string& name)
 : PointcloudMapper(options, name)
{
	declare_parameter("vdb_resolution", 0.1);
	declare_parameter("vdb_publish_map", false);

    declare_parameter<bool>("fast_mode", false);
    get_parameter("fast_mode", mVdbConfig.fast_mode);
    declare_parameter<double>("accumulation_period", 1);
    get_parameter("accumulation_period", mVdbConfig.accumulation_period);

    declare_parameter<double>("max_range", 10.0);
    get_parameter("max_range", mVdbConfig.max_range);
    declare_parameter<double>("prob_hit", 0.7);
    get_parameter("prob_hit", mVdbConfig.prob_hit);
    declare_parameter<double>("prob_miss", 0.4);
    get_parameter("prob_miss", mVdbConfig.prob_miss);
    declare_parameter<double>("prob_thres_min", 0.12);
    get_parameter("prob_thres_min", mVdbConfig.prob_thres_min);
    declare_parameter<double>("prob_thres_max", 0.97);
    get_parameter("prob_thres_max", mVdbConfig.prob_thres_max);
    declare_parameter<std::string>("map_directory_path", "");
    get_parameter("map_directory_path", mVdbConfig.map_directory_path);

	mVdbMapping = std::make_shared<vdb_mapping::OccupancyVDBMapping>(get_parameter("vdb_resolution").as_double());

	mVdbMapPublisher = create_publisher<visualization_msgs::msg::Marker>("vdb_map", 1);
	mVdbMapping->setConfig(mVdbConfig);
	mVdbMapping->addInputSource(mPclSensor->getName(), 0, 0);
	
	mGenerateMapService = create_service<std_srvs::srv::Empty>("generate_map",
		std::bind(&VDBMapper::generateMap, this, std::placeholders::_1, std::placeholders::_2));

	// Add all imported measurements to the VDB map
	generateMap({}, {});
}

void VDBMapper::generateMap(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                  std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
	mVdbMapping->resetMap();
	for(const auto& v : mGraph->getVerticesByType("slam3d::PointCloudMeasurement"))
	{		
		PointCloudMeasurement::Ptr m =
			boost::dynamic_pointer_cast<PointCloudMeasurement>(mGraph->getMeasurement(v.measurementUuid));
		addScanToMap(m->getPointCloud(), v.correctedPose * m->getSensorPose());
		mVdbMapping->integrateUpdate();
	}	
	
	mLogger->message(INFO, (boost::format("VDB map has %1% active voxels.")%  mVdbMapping->getGrid()->activeVoxelCount()).str());
	sendMap();
}

void VDBMapper::sendMap()
{
	visualization_msgs::msg::Marker visualization_marker_msg;
	sensor_msgs::msg::PointCloud2 cloud_msg;
	nav_msgs::msg::OccupancyGrid occupancy_grid_msg;
	VDBMappingTools<vdb_mapping::OccupancyVDBMapping>::createMappingOutput(
		mVdbMapping->getGrid(),
		mMapFrame,
		visualization_marker_msg,
		cloud_msg,
		occupancy_grid_msg,
		true,  // m_publish_vis_marker,
		false, // m_publish_pointcloud,
		false, // m_publish_occupancy_grid,
		-10, // m_lower_visualization_z_limit,
		10, // m_upper_visualization_z_limit,
		get_parameter("vdb_resolution").as_double(), // m_resolution,
		1.0  // m_two_dim_projection_threshold
	);
	mVdbMapPublisher->publish(visualization_marker_msg);
}

void VDBMapper::addScanToMap(const PointCloud::ConstPtr scan, const Transform& pose)
{
	PointCloud::Ptr tempCloud(new PointCloud);
	pcl::transformPointCloud(*scan, *tempCloud, pose.matrix());
	mVdbMapping->addDataToAccumulate(tempCloud, pose.translation(), mPclSensor->getName());
	
	if(get_parameter("vdb_publish_map").as_bool())
	{
		sendMap();
	}
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(slam3d::VDBMapper)
