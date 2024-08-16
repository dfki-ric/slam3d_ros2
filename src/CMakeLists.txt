# Public library with generic ROS functions
add_library(${PROJECT_NAME} SHARED
	TfOdometry.cpp
	RosClock.cpp
	GraphPublisher.cpp
	PointcloudMapper.cpp
#	GpsPublisher.cpp
#	LoopCloser.cpp
)

target_include_directories(${PROJECT_NAME}
	PUBLIC
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
	"rclcpp"
	"rclcpp_components"
	"tf2_ros"
	"tf2_eigen"
	"sensor_msgs"
	"visualization_msgs"
	"std_srvs"
	"pcl_conversions"
)

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
