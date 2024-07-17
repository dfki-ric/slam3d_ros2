#pragma once

#include <slam3d/sensor/pcl/PointCloudSensor.hpp>

namespace slam3d
{
	RegistrationParameters declareRegistrationParams(rclcpp::Node* node, const std::string& name)
	{
		RegistrationParameters params;
		params.correspondence_randomness =
			node->declare_parameter(name+".correspondence_randomness", params.correspondence_randomness);

		params.euclidean_fitness_epsilon =
			node->declare_parameter(name+".euclidean_fitness_epsilon", params.euclidean_fitness_epsilon);

		params.max_correspondence_distance =
			node->declare_parameter(name+".max_correspondence_distance", params.max_correspondence_distance);

		params.max_fitness_score =
			node->declare_parameter(name+".max_fitness_score", params.max_fitness_score);

		params.maximum_iterations =
			node->declare_parameter(name+".maximum_iterations", params.maximum_iterations);

		params.maximum_optimizer_iterations =
			node->declare_parameter(name+".maximum_optimizer_iterations", params.maximum_optimizer_iterations);

		params.outlier_ratio =
			node->declare_parameter(name+".outlier_ratio", params.outlier_ratio);

		params.point_cloud_density =
			node->declare_parameter(name+".point_cloud_density", params.point_cloud_density);
			
		std::string registration_algorithm = node->declare_parameter(name+".registration_algorithm", "GICP");
		if(registration_algorithm == "ICP")
			params.registration_algorithm = ICP;
		else if(registration_algorithm == "GICP")
			params.registration_algorithm = GICP;
		else if(registration_algorithm == "GICP_OMP")
			params.registration_algorithm = GICP_OMP;
		else if(registration_algorithm == "NDT")
			params.registration_algorithm = NDT;
		else if(registration_algorithm == "NDT_OMP")
			params.registration_algorithm = NDT_OMP;
		else
			std::cerr << "Got invalid registration algorithm: " << registration_algorithm << std::endl;

		params.resolution =
			node->declare_parameter(name+".resolution", params.resolution);

		params.rotation_epsilon =
			node->declare_parameter(name+".rotation_epsilon", params.rotation_epsilon);

		params.step_size =
			node->declare_parameter(name+".step_size", params.step_size);

		params.transformation_epsilon =
			node->declare_parameter(name+".transformation_epsilon", params.transformation_epsilon);
		
		return params;
	}
	
	class RosPclSensor : public PointCloudSensor
	{
	public:
		RosPclSensor(const std::string& name, Logger* logger, rclcpp::Node* node)
		: PointCloudSensor(name, logger)
		{
			node->declare_parameter(name+".scan_resolution", 0.1);
			node->declare_parameter(name+".map_resolution", 0.1);
			node->declare_parameter(name+".pose_translation", 0.5);
			node->declare_parameter(name+".pose_rotation", 0.5);

			node->declare_parameter(name+".neighbor_radius", 5.0);
			node->declare_parameter(name+".max_neighbor_links", 1);
			node->declare_parameter(name+".patch_building_range", 0);
			node->declare_parameter(name+".min_loop_length", 10);
			node->declare_parameter(name+".link_previous", true);

			setMinPoseDistance(
				node->get_parameter(name+".pose_translation").as_double(),
				node->get_parameter(name+".pose_rotation").as_double());

			setScanResolution(
				node->get_parameter(name+".scan_resolution").as_double());

			setMapResolution(
				node->get_parameter(name+".map_resolution").as_double());

			setNeighborRadius(
				node->get_parameter(name+".neighbor_radius").as_double(),
				node->get_parameter(name+".max_neighbor_links").as_int());

			setPatchBuildingRange(
				node->get_parameter(name+".patch_building_range").as_int());

			setMinLoopLength(
				node->get_parameter(name+".min_loop_length").as_int());

			setLinkPrevious(
				node->get_parameter(name+".link_previous").as_bool());

			setRegistrationParameters(declareRegistrationParams(node, name+".registration"), false);
		}
	};
}
