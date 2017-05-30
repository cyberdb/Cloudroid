FROM ros:my
COPY ./temp/install /catkin_install
COPY ./temp/ros_entry.sh /
COPY ./rosdep.py /
COPY ./buildimages.sh /

RUN chmod +x /ros_entry.sh
RUN chmod +x /rosdep.py
#RUN /bin/bash -c 'chmod +x /catkin_install/devel/_setup_util.py; exit 0'
#RUN /bin/bash -c 'chmod +x /catkin_install/install_isolated/_setup_util.py; exit 0'
RUN chmod +x /buildimages.sh

RUN ./rosdep.py


ENTRYPOINT ["/ros_entry.sh"]

