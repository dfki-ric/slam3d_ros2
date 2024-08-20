cmake_minimum_required(VERSION 3.16.3)
project(slam3d_ros2)

# find dependencies
find_package(slam3d 3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Public library with generic ROS functions
add_library(${PROJECT_NAME} SHARED
	src/TfOdometry.cpp
	src/RosClock.cpp
	src/GraphPublisher.cpp
	src/PointcloudMapper.cpp
#	src/GpsPublisher.cpp
#	src/LoopCloser.cpp
)

target_include_directories(${PROJECT_NAME}
	PUBLIC
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
)

set(AMENT_DEPS
	"rclcpp"
	"rclcpp_components"
	"tf2_ros"
	"tf2_eigen"
	"sensor_msgs"
	"visualization_msgs"
	"std_srvs"
	"pcl_conversions"
)
ament_target_dependencies(${PROJECT_NAME} ${AMENT_DEPS})

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "slam3d::PointcloudMapper"
    EXECUTABLE pointcloud_mapper
)

target_link_libraries(${PROJECT_NAME}
	${PCL_COMMON_LIBRARIES}
	slam3d::solver-g2o
	slam3d::graph-boost
	slam3d::sensor-pcl
)

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
ament_export_dependencies(${AMENT_DEPS})
ament_package()
