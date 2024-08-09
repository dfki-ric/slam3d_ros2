cmake_minimum_required(VERSION 3.16.3)
project(slam3d_ros2)

# find dependencies
find_package(slam3d 3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Needed by pcl_conversions, but not properly exported in foxy
find_package(PCL 1.7 REQUIRED COMPONENTS common)

add_subdirectory(src)
add_subdirectory(node)

ament_export_targets(slam3d_ros2_exports HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp visualization_msgs slam3d)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

