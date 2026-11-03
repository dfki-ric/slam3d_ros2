cmake_minimum_required(VERSION 3.16.3)
project(slam3d_ros2)

# find dependencies
find_package(slam3d 3 REQUIRED)
find_package(slam3d_octomap REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(vdb_mapping_ros2)

set(SOURCE_FILES
	src/TfOdometry.cpp
	src/TfGravity.cpp
	src/RosClock.cpp
	src/RosPclSensor.cpp
	src/GraphPublisher.cpp
	src/PointcloudMapper.cpp
	src/PlyLoader.cpp
#	src/GpsPublisher.cpp
#	src/LoopCloser.cpp
)

# Public library with generic ROS functions
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})

target_include_directories(${PROJECT_NAME}
	PUBLIC
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
		${pcl_conversions_INCLUDE_DIRS}
		${pcl_ros_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
	PUBLIC
		rclcpp_components::component
		tf2_ros::tf2_ros
		tf2_eigen::tf2_eigen
		sensor_msgs::sensor_msgs
		visualization_msgs::visualization_msgs
		std_srvs::std_srvs
		octomap_msgs::octomap_msgs
		${pcl_conversions_LIBRARIES}
		${pcl_ros_LIBRARIES}
		slam3d::solver-g2o
		slam3d::graph-boost
		slam3d::sensor-pcl
		slam3d::serialization
	PRIVATE
		slam3d_octomap::slam3d_octomap
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "slam3d::PlyLoader"
    EXECUTABLE ply_loader
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "slam3d::PointcloudMapper"
    EXECUTABLE pointcloud_mapper
)

if(${vdb_mapping_ros2_FOUND})
target_link_libraries(${PROJECT_NAME}
	PRIVATE
		vdb_mapping_ros2::vdb_mapping_ros2
)
rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "slam3d::VDBMapper"
    EXECUTABLE vdb_mapper
)
endif()

install(
	TARGETS ${PROJECT_NAME}
	EXPORT ${PROJECT_NAME}_exports
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin
)

install(
	DIRECTORY ${PROJECT_SOURCE_DIR}/include/
	DESTINATION include/${PROJECT_NAME}
	FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY launch config
	DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(slam3d_ros2_exports HAS_LIBRARY_TARGET)
ament_export_dependencies(
	slam3d
	ament_cmake
	rclcpp_components
	tf2_ros
	tf2_eigen
	sensor_msgs
	visualization_msgs
	std_srvs
	octomap_msgs
)
ament_package()
