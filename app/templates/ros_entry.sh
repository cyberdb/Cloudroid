#!/bin/bash
set -e
# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/catkin_install/devel/setup.bash"
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/catkin_install/devel/src
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/catkin_install/src


nohup roslaunch rosbridge_server rosbridge_websocket.launch > /dev/null &

{% for cmd in start_cmds %}
{{ cmd }}
{% endfor %}
