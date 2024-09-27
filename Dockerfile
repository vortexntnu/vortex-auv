# Base image: using ROS 2 Humble
FROM ros:humble

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    git \
    && rm -rf /var/lib/apt/lists/*

# Set up the workspace
WORKDIR /ros2_ws

# Copy the contents of vortex-auv into the container
COPY . src/vortex-auv

# Clone vortex-msgs repo into the src folder
RUN git clone https://github.com/vortexntnu/vortex-msgs.git src/vortex-msgs

# Source the ROS 2 environment
SHELL ["/bin/bash", "-c"]

# Build the workspace using colcon
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# Source the workspace on container start
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source install/setup.bash && bash"]