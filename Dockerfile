FROM ros:kinetic-ros-core-xenial
WORKDIR /catkin_ws
RUN apt-get update
RUN apt-get install --yes python-catkin-tools python-wstool
COPY . ./src/manta-auv

WORKDIR /catkin_ws/src
RUN wstool init
RUN wstool merge manta-auv/dependencies.rosinstall
RUN wstool up

WORKDIR /catkin_ws
RUN rosdep update
RUN rosdep install --default-yes --from-paths src --ignore-src --rosdistro kinetic

# See https://stackoverflow.com/questions/20635472/using-the-run-instruction-in-a-dockerfile-with-source-does-not-work
RUN ["/bin/bash", "-c", "source /opt/ros/kinetic/setup.bash && \
catkin build"]
