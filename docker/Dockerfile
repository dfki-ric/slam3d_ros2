
FROM ros:jazzy

ENV ROS_WORKSPACE_ROOT=/opt/ws

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-jazzy-visualization-msgs \
    ros-jazzy-libnabo \
    ros-jazzy-libg2o \
    ros-jazzy-pcl-ros \
    ros-jazzy-pcl-conversions \
    ros-jazzy-octomap \
    ros-jazzy-octomap-msgs \
    ros-jazzy-rmw-cyclonedds-cpp \
    && apt-get clean

COPY ws.repos colcon_defaults.yaml ${ROS_WORKSPACE_ROOT}/

RUN cd ${ROS_WORKSPACE_ROOT} \
    && vcs import < ws.repos \
    && (. /opt/ros/jazzy/setup.bash; colcon build)
