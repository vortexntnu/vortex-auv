FROM ros:noetic

ARG distro=noetic
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"] 

# ROS package dependencies
RUN apt update && \
    apt install -y --no-install-recommends \
    apt-utils \
    net-tools \     
    tcpdump \ 
    nano \
    ros-$distro-roslint \
    ros-$distro-move-base-msgs \
    ros-$distro-tf \
    ros-$distro-tf2 \
    ros-$distro-eigen-conversions \
    ros-$distro-joy \
    ros-$distro-tf2-geometry-msgs \
    ros-$distro-pcl-ros \
    ros-$distro-rviz \
    ros-$distro-rosbridge-server \
    ros-$distro-message-to-tf \
    ros-$distro-geographic-msgs \
    ros-$distro-move-base \
    ros-$distro-smach-ros \
    ros-$distro-tf-conversions \
    python3-osrf-pycommon \
    python3-openpyxl \
    python3-yaml \
    python3-pip \
    python3-wheel \
    python3-catkin-tools \
    python3-vcstool \
    python3-pandas \
    python3-scipy \
    libgeographic-dev \
    libeigen3-dev \
    libglfw3-dev \
    libglew-dev \
    libjsoncpp-dev \
    libtclap-dev \
    protobuf-compiler

RUN pip install \
    rospkg \
    pyquaternion \
    quadprog \
    sklearn \
    board \
    busio \
    enum34

# Here we assume that the root contains vortex-auv, vortex-msgs and robot_localization
COPY ./vortex-auv /vortex_ws/src/vortex-auv
COPY ./vortex-msgs /vortex_ws/src/vortex-msgs 
COPY ./robot_localization /vortex_ws/src/robot_localization

RUN source /opt/ros/$distro/setup.bash && cd /vortex_ws && catkin build

COPY ./vortex-auv/docker/entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]
