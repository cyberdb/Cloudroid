FROM ros:my
COPY ./temp/install /catkin_install
COPY ./rosdep.py /
COPY ./buildimages.sh /

RUN chmod +x /rosdep.py
RUN chmod +x /catkin_install/devel/_setup_util.py
RUN chmod +x /buildimages.sh


RUN ./rosdep.py
