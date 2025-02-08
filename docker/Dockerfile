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
    # Ensure the ROS 2 APT repository is properly set up.
    # ------------------------------------------------------------------------------
    ARG ROS_DISTRO
    ENV ROS_DISTRO=${ROS_DISTRO}
    
    # Install essential tools for repository setup, remove conflicting files,
    # and then add the ROS 2 repository using the key already provided by the base image.
    RUN apt-get update && \
        apt-get install -y curl gnupg lsb-release && \
        rm -f /etc/apt/sources.list.d/ros2-latest.list && \
        rm -f /etc/apt/sources.list.d/ros2.list && \
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros2-latest-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list && \
        apt-get update
    
    # ------------------------------------------------------------------------------
    # Install ROS packages.
    # ------------------------------------------------------------------------------
    RUN apt-get install -y \
            git \
            python3-rosdep \
            python3-vcstool \
            ros-${ROS_DISTRO}-ros-core \
            python3-pip && \
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
    