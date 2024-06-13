cmake_minimum_required(VERSION 3.16.3)
project(slam3d_ros2)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(slam3d 3 REQUIRED)

# Needed by pcl_conversions, but not properly exported in foxy
find_package(PCL 1.7 REQUIRED COMPONENTS common)

# Public library with generic ROS functions
add_library(${PROJECT_NAME} SHARED
	src/RosClock.cpp
	src/GraphPublisher.cpp
#	src/GpsPublisher.cpp
#	src/LoopCloser.cpp
)

target_include_directories(${PROJECT_NAME}
	PUBLIC
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
	"rclcpp"
	"visualization_msgs"
)

target_link_libraries(${PROJECT_NAME}
	slam3d::core
)

install(
	TARGETS ${PROJECT_NAME}
	EXPORT ${PROJECT_NAME}_exports
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin
)

install(
	DIRECTORY include/
	DESTINATION include/${PROJECT_NAME}
	FILES_MATCHING PATTERN "*.hpp"
)

# Simple mapper node to create a pointcloud map
add_executable(pointcloud_mapper
	src/pointcloud_mapper.cpp
)

ament_target_dependencies(pointcloud_mapper
	"rclcpp"
	"tf2_ros"
	"tf2_eigen"
	"sensor_msgs"
	"std_srvs"
	"pcl_conversions"
)

target_link_libraries(pointcloud_mapper
	${PCL_COMMON_LIBRARIES}
	slam3d::solver-g2o
	slam3d::graph-boost
	slam3d::sensor-pcl
	${PROJECT_NAME}
)

install(
	TARGETS pointcloud_mapper
	DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(slam3d_ros2_exports HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp visualization_msgs slam3d)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
