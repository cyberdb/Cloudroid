#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/catkin_install/devel/setup.bash" || true
source "/catkin_install/install_isolated/setup.bash" || true
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/catkin_install/devel/src
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/catkin_install/install_isolated/src
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/catkin_install/src


apt-get update

rosdep install --from-paths /catkin_install/src --ignore-src --rosdistro=$ROS_DISTRO -y