add_subdirectory(visualization)

add_executable(pointcloud_mapper
	pointcloud_mapper.cpp
)

target_link_libraries(pointcloud_mapper PUBLIC
	rclcpp::rclcpp
	tf2_ros::tf2_ros
	${sensor_msgs_TARGETS}
	${std_srvs_TARGETS}
	${PCL_COMMON_LIBRARIES}
	slam3d::solver-g2o
	slam3d::graph-boost
	slam3d::sensor-pcl
	visualization
)

install(
	TARGETS pointcloud_mapper
	DESTINATION lib/${PROJECT_NAME}
)
