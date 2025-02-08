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
    # Use the ROS 2 sources already set in the base image; update apt and install needed packages.
    # ------------------------------------------------------------------------------
    ARG ROS_DISTRO
    ENV ROS_DISTRO=${ROS_DISTRO}
    
    RUN apt-get update && \
        apt-get install -y \
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
    