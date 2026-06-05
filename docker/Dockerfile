
FROM ros:jazzy

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

SHELL ["/bin/bash", "-c"]

USER ubuntu
WORKDIR /home/ubuntu

COPY ws.repos colcon_defaults.yaml .

RUN vcs import < ws.repos \
    && (. /opt/ros/jazzy/setup.bash; colcon build)
