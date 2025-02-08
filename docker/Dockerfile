# ------------------------------------------------------------------------------
# Define a base image that can be overridden at build time.
# ------------------------------------------------------------------------------
    ARG BASE_IMAGE=ros:humble-ros-base

    #######################################
    # Stage: dependencies
    #######################################
    FROM ${BASE_IMAGE} AS dependencies
    ARG TARGETARCH
    # Set the TARGETARCH environment variable for later use.
    ENV TARGETARCH=${TARGETARCH}

    USER root
    SHELL ["/bin/bash", "-c"]
    ARG DEBIAN_FRONTEND=noninteractive

    # ------------------------------------------------------------------------------
    # Create the workspace folder structure.
    # WORKSPACE: The main directory for your build process.
    # ------------------------------------------------------------------------------
    ENV WORKSPACE=/docker/ws
    WORKDIR $WORKSPACE
    RUN mkdir -p src

    # ------------------------------------------------------------------------------
    # Setup ROS keys and package sources.
    # This installs curl and gnupg, then adds the ROS 2 repository if it isn't already present.
    # ------------------------------------------------------------------------------
    ARG ROS_DISTRO
    ENV ROS_DISTRO=${ROS_DISTRO}
    RUN test -n "$ROS_DISTRO" || (echo "missing build-arg: ROS_DISTRO" && false) && \
        apt-get update && \
        apt-get install -y curl gnupg && \
        if ! grep -q "http://packages.ros.org/ros2/ubuntu" /etc/apt/sources.list /etc/apt/sources.list.d/*; then \
            curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
            echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null ; \
        fi && \
        rm -rf /var/lib/apt/lists/*

    # ------------------------------------------------------------------------------
    # Install basic build tools and optionally ROS.
    # This step installs git, rosdep, and vcstool.
    # If ROS installation is not disabled, it also installs ros-core.
    # ------------------------------------------------------------------------------
    ARG DISABLE_ROS_INSTALLATION="false"
    RUN apt-get update && \
        apt-get install -y \
            git \
            python3-rosdep \
            python3-vcstool && \
        if [[ "$DISABLE_ROS_INSTALLATION" != "true" ]]; then \
            apt-get install -y ros-${ROS_DISTRO}-ros-core; \
        fi && \
        rm -rf /var/lib/apt/lists/*

    # ------------------------------------------------------------------------------
    # Copy the repository contents into the image under the "src" directory.
    # ------------------------------------------------------------------------------
    COPY . src

    # ------------------------------------------------------------------------------
    # Install python3-pip so that pip3 is available for installing Python packages.
    # ------------------------------------------------------------------------------
    RUN apt-get update && apt-get install -y python3-pip

    # ------------------------------------------------------------------------------
    # Ensure that the "pip" command is available by linking pip3 to pip.
    # This is needed because some rosdep rules expect the "pip" command.
    # ------------------------------------------------------------------------------
    RUN ln -sf "$(which pip3)" /usr/bin/pip

    # ------------------------------------------------------------------------------
    # Accept an optional build argument for extra dependency installation commands.
    # If INSTALL_CMD is provided (non-empty), execute it to install extra dependencies.
    # ------------------------------------------------------------------------------
        ARG INSTALL_CMD=""
        RUN if [ -n "$INSTALL_CMD" ]; then \
              echo "Running custom install commands:"; \
              echo "$INSTALL_CMD"; \
              eval "$INSTALL_CMD"; \
            else \
              echo "No custom install commands provided."; \
            fi

    # ------------------------------------------------------------------------------
    # Run rosdep to install system dependencies for ROS packages.
    # This step updates rosdep's cache and installs any missing system dependencies
    # for all ROS packages found in the "src" directory (ignoring source-only packages).
    # ------------------------------------------------------------------------------
    RUN rosdep update && \
        rosdep install --from-paths src --ignore-src -r -y

    #######################################
    # Stage: dev
    #######################################
    FROM dependencies AS dev
    # Set the default command to start a bash shell when the container runs.
    CMD ["bash"]

