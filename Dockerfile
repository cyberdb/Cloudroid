FROM ros:my
COPY ./temp/install /catkin_install
COPY ./temp/ros_entry.sh /
COPY ./rosdep.py /
COPY ./buildimages.sh /
RUN chmod +x /ros_entry.sh
RUN chmod +x /rosdep.py
RUN chmod +x /catkin_install/devel/_setup_util.py
RUN chmod +x /buildimages.sh

RUN ./buildimages.sh
RUN ./rosdep.py
ENTRYPOINT ["/ros_entry.sh"]


