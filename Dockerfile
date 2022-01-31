FROM tiryoh/ros-melodic-desktop

ARG distro=melodic
ENV DEBIAN_FRONTEND=noninteractive

# Create vortex user
RUN useradd -ms /bin/bash \
    --home /home/vortex  vortex
RUN echo "vortex:vortex" | chpasswd
RUN usermod -aG sudo vortex

# ROS package dependencies
RUN apt update && \
    apt install -y \
    apt-utils \
    ros-$distro-roslint \
    ros-$distro-move-base-msgs \
    ros-$distro-tf \
    ros-$distro-tf2 \
    ros-$distro-eigen-conversions \
    libeigen3-dev \
    ros-$distro-joy \
    ros-$distro-tf2-geometry-msgs \
    ros-$distro-pcl-ros \
    ros-$distro-rviz \
    libeigen3-dev \
    libglfw3-dev \
    libglew-dev \
    libjsoncpp-dev \
    libtclap-dev \
    net-tools \     
    tcpdump \ 
    nano \ 
    protobuf-compiler \
    ros-$distro-rosbridge-server \
    ros-$distro-message-to-tf \
    ros-$distro-geographic-msgs \
    ros-$distro-move-base \
    python-osrf-pycommon \
    python-openpyxl \
    python-yaml \
    python-enum34 \
    python-pip \
    python-catkin-tools \
    libgeographic-dev

RUN pip install \
    rospkg \
    pyquaternion \
    vcstool \
    pandas \
    quadprog \
    scipy

RUN echo "source /opt/ros/melodic/setup.bash" >> /home/vortex/.bashrc
RUN echo "source /home/vortex/auv_ws/devel/setup.bash" >> /home/vortex/.bashrc

RUN mkdir -p /home/vortex/auv_ws/src
RUN chown -R vortex: /home/vortex/auv_ws
RUN chmod a+rw /dev/ttyUSB* ; exit 0

CMD ["/bin/bash"]