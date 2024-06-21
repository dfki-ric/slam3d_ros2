# Simple mapper node to create a pointcloud map
add_executable(pointcloud_mapper
	pointcloud_mapper.cpp
)

target_link_libraries(pointcloud_mapper
	${PROJECT_NAME}
)

install(
	TARGETS pointcloud_mapper
	DESTINATION lib/${PROJECT_NAME}
)
