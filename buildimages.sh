#!/bin/bash
set -e
# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/catkin_install/devel/setup.bash"
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/catkin_install/devel/src
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/catkin_install/src


i=1
while [ $? -ne 0 ] || [ "$i" != "5" ]
do 
	i=$(($i+1))
	apt-get update
done


./rosdep.py

