add_library(visualization SHARED
	GraphPublisher.cpp
#	GpsPublisher.cpp
#	LoopCloser.cpp
)

target_include_directories(visualization
	PUBLIC
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
		$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(visualization
	PUBLIC
		slam3d::core
		rclcpp::rclcpp
		${visualization_msgs_TARGETS}
)

install(
	TARGETS visualization
	EXPORT slam3d_ros2_exports
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin
)

install(
	DIRECTORY .
	DESTINATION include/${PROJECT_NAME}
	FILES_MATCHING PATTERN "*.hpp"
)
