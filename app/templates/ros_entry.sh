#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/catkin_install/setup.bash"
nohup roslaunch rosbridge_server rosbridge_websocket.launch > /dev/null &

{% for cmd in start_cmds %}
{{ cmd }}
{% endfor %}
