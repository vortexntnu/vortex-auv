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
    git \ 
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
    enum34

# Here we assume that the root of vortex-auv is the context
COPY . /vortex_ws/src/vortex-auv 

# When the Docker image is eventually built by a GitHub action, this can be a COPY instead
RUN cd /vortex_ws/src && git clone https://github.com/vortexntnu/robot_localization
RUN cd /vortex_ws/src && git clone https://github.com/vortexntnu/vortex-msgs
RUN source /opt/ros/$distro/setup.bash && cd /vortex_ws && catkin build

COPY ./docker/entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]