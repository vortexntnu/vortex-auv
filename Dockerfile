# Stage 1: Build the ROS 2 workspace
FROM ros:humble-ros-base AS builder

# Install necessary dependencies for building
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

# Source the ROS 2 environment and build the workspace using colcon
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# Stage 2: Minimal runtime image with the install directory only
FROM ros:humble-ros-core

# Set up the workspace
WORKDIR /ros2_ws

# Copy only the necessary files from the install directory
COPY --from=builder /ros2_ws/install /ros2_ws/install
COPY --from=builder /ros2_ws/build /ros2_ws/build

# Source the workspace on container start
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source install/setup.bash && bash"]
