# ------------------------------------------------------------------------------
# Define a base image that can be overridden at build time.
# ------------------------------------------------------------------------------
    ARG BASE_IMAGE=ros:humble-ros-base

    #######################################
    # Single Stage
    #######################################
    FROM ${BASE_IMAGE}

    USER root
    SHELL ["/bin/bash", "-c"]
    ARG DEBIAN_FRONTEND=noninteractive

    # ------------------------------------------------------------------------------
    # Create the workspace folder structure.
    # ------------------------------------------------------------------------------
    ENV WORKSPACE=/docker/ws
    WORKDIR $WORKSPACE

    # ------------------------------------------------------------------------------
    # Update apt and install required dependencies.
    # ------------------------------------------------------------------------------
    ARG ROS_DISTRO
    ENV ROS_DISTRO=${ROS_DISTRO}

    RUN apt-get update && \
        apt-get install -y \
            git \
            python3-rosdep \
            python3-vcstool \
            ros-${ROS_DISTRO}-ros-core \
            python3-pip

    # ------------------------------------------------------------------------------
    # Clone the vortex-auv repository and fetch submodules.
    # ------------------------------------------------------------------------------
    COPY . .


    # ---------------------z---------------------------------------------------------
    # Run rosdep to install all missing dependencies.
    # ------------------------------------------------------------------------------
    RUN rosdep update && rosdep install --from-paths . --ignore-src -r -y

    # ------------------------------------------------------------------------------
    # Default command
    # ------------------------------------------------------------------------------
    CMD ["bash"]

