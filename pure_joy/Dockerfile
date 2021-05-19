FROM osrf/ros:noetic-desktop AS project-sources

ENV DEBIAN_FRONTEND=noninteractive
ENV QT_X11_NO_MITSHM 1

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

RUN apt-get update && apt-get install -y \
    cmake \
    libgl1-mesa-glx \
    libsodium-dev \
    libzmq3-dev \
    sudo git nano wget \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp
# get cppzmq bindings
RUN wget https://github.com/zeromq/cppzmq/archive/v4.7.1.tar.gz -O cppzmq-4.7.1.tar.gz \
  && tar -xzf cppzmq-4.7.1.tar.gz \
  && cd cppzmq-4.7.1 \
  && mkdir build \
  && cd build \
  && cmake .. -DCPPZMQ_BUILD_TESTS=OFF \
  && make -j4 install

WORKDIR /
RUN rm -rf /tmp/*
RUN ldconfig

FROM project-sources AS ros-user

ARG UID=1000
ARG GID=1000
RUN addgroup --gid ${GID} ros
RUN adduser --gecos "ROS User" --disabled-password --uid ${UID} --gid ${GID} ros
RUN usermod -a -G dialout ros
RUN echo "ros ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/99_aptget
RUN chmod 0440 /etc/sudoers.d/99_aptget && chown root:root /etc/sudoers.d/99_aptget

USER ros

ENV HOME /home/ros

# workspace setup
RUN mkdir -p ~/ros_ws/src

RUN cd ~/ros_ws/src && /bin/bash -c "source /ros_entrypoint.sh; catkin_init_workspace"
WORKDIR ${HOME}/ros_ws/src
RUN git clone https://github.com/ros-drivers/joystick_drivers.git && mv joystick_drivers/joy ./joy \
   && rm -r joystick_drivers
RUN cd ${HOME}/ros_ws && /bin/bash -c "source /ros_entrypoint.sh; catkin_make"

# Change .bashrc
COPY docker/update_bashrc /sbin/update_bashrc
RUN sudo chmod +x /sbin/update_bashrc ; sudo chown ros /sbin/update_bashrc ; sync ; /bin/bash -c /sbin/update_bashrc ; sudo rm /sbin/update_bashrc

# Change entrypoint to source ~/.bashrc and start in ~
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN sudo chmod +x /ros_entrypoint.sh ; sudo chown ros /ros_entrypoint.sh ;

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]