FROM ros:my
COPY ./temp/install /catkin_install
COPY ./temp/ros_entry.sh /
RUN chmod +x /ros_entry.sh
RUN chmod +x /catkin_install/_setup_util.py

ENTRYPOINT ["/ros_entry.sh"]


