# Docker

## Prerequisites
- Install [Docker](https://www.docker.com/get-started)
  - On macOS and Windows, Docker Desktop is recommended.
  - On Linux, install Docker Engine via your distribution’s package manager.
- Standard ROS 2 Workspace Structure
  - The scripts assume your local file structure follows a standard ROS 2 workspace layout (this will ensure build paths and mounting function as expected):
```bash
ros_ws/
│── build/
│── log/
│── install/
│── src/
│   └── vortex-auv/
│   └── vortex-msgs/
```

## Building and Running
1. Clone this repository (if you haven’t already) so you have the local workspace on your machine.
2. Go to the docker/ folder in the repository.
3. Build the Docker image:
```bash
cd docker
./build.sh
```
This will create (and load) the Docker image locally, tagged by default as auv-image:latest.

4. Run the container:
```bash
./run.sh
```
  - This starts an interactive container using the just-built image.
  - Your local repository is mounted into /ros_ws inside the container, so any changes on your host machine are instantly visible in the container.
Once you’re in the container’s shell, you can navigate to /ros_ws and run ROS 2 commands as usual. For instance:
```bash
cd /ros_ws
colcon build
colcon test
ros2 launch my_package my_launch_file.launch.py
```
Because your local workspace is mounted, the generated build/, log/, and install/ folders remain on your host:
```bash
ros_ws/
│── build/
│── log/
│── install/
│── src/
│   └── vortex-auv/
│   └── vortex-msgs/
```
This setup ensures that code changes on your host are immediately reflected in the container, and any builds done inside the container are accessible back on your host.
