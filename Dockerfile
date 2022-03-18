FROM ros:melodic

ARG distro=melodic
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
    python-osrf-pycommon \
    python-openpyxl \
    python-yaml \
    python-enum34 \
    python-pip \
    python-catkin-tools \
    python-vcstool \
    libgeographic-dev \
    libeigen3-dev \
    libglfw3-dev \
    libglew-dev \
    libjsoncpp-dev \
    libtclap-dev \
    protobuf-compiler

RUN pip install \
    wheel \
    rospkg \
    pyquaternion \
    pandas \
    quadprog \
    scipy \
    sklearn

COPY . /vortex_ws/src/Vortex-AUV
RUN cd /vortex_ws/src && git clone https://github.com/vortexntnu/robot_localization
RUN source /opt/ros/$distro/setup.bash && cd /vortex_ws && catkin build

COPY ./entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]