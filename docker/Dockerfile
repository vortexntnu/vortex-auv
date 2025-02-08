# ------------------------------------------------------------------------------
# Define a base image that can be overridden at build time.
# ------------------------------------------------------------------------------
    ARG BASE_IMAGE=ros:humble-ros-base

    #######################################
    # Stage: dependencies
    #######################################
    FROM ${BASE_IMAGE} AS dependencies

    USER root
    SHELL ["/bin/bash", "-c"]
    ARG DEBIAN_FRONTEND=noninteractive

    # ------------------------------------------------------------------------------
    # Create the workspace folder structure.
    # ------------------------------------------------------------------------------
    ENV WORKSPACE=/docker/ws
    WORKDIR $WORKSPACE
    RUN mkdir -p src

    # ------------------------------------------------------------------------------
    # Setup ROS keys and package sources.
    # ------------------------------------------------------------------------------
    ARG ROS_DISTRO
    ENV ROS_DISTRO=${ROS_DISTRO}
    RUN apt-get update && \
        apt-get install -y \
            curl gnupg git python3-rosdep python3-vcstool ros-${ROS_DISTRO}-ros-core python3-pip && \
        if ! grep -q "http://packages.ros.org/ros2/ubuntu" /etc/apt/sources.list /etc/apt/sources.list.d/*; then \
            curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
            echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null ; \
        fi && \
        rm -rf /var/lib/apt/lists/*

    # ------------------------------------------------------------------------------
    # Copy the repository contents into the image under the "src" directory.
    # ------------------------------------------------------------------------------
    COPY . src

    #######################################
    # Stage: dev
    #######################################
    FROM dependencies AS dev
    CMD ["bash"]

