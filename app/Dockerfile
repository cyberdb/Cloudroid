FROM ros:my
COPY ./temp/install /catkin_install
COPY ros_entry.sh /
RUN chmod +x /ros_entry.sh

ENTRYPOINT ["/ros_entry.sh"]
