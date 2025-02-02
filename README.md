## Vortex AUV
[![Industrial CI](https://github.com/vortexntnu/vortex-auv/actions/workflows/industrial-ci.yaml/badge.svg)](https://github.com/vortexntnu/vortex-auv/actions/workflows/industrial-ci.yaml)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/vortexntnu/vortex-auv/main.svg)](https://results.pre-commit.ci/latest/github/vortexntnu/vortex-auv/main)

![Banner](docs/banner_image.png)

This repo contains software for operating UUVs, developed by students at NTNU. The software is based on the ROS2 Humble framework, and aims to be hardware independent. Although the main focus of Vortex is autonomous operation, this software stack supports both AUV and ROV operations.


## Docker
This project uses the [docker-ros](https://github.com/ika-rwth-aachen/docker-ros) repository for building and managing Docker images. The docker-ros repository is included as a Git submodule and is configured to build images locally.

### Prerequisites
- Install [Docker](https://www.docker.com/get-started)

### Cloning
To clone this repository with the docker-ros submodule, use the following command:
```bash
git clone --recurse-submodules https://github.com/vortexntnu/vortex-asv.git
```
Alternatively, if you have already cloned the repository, run the following command to initialize the submodule:
```bash
git submodule update --init --recursive
```
### Building and Running
1. Run the following command from the root of this repository to build the Docker image, its called ```asv-image:latest```, and start a container:
```bash
./entrypoint.sh
```
2. The Docker container will start with the following configurations:
- The source code from this repository is mounted to ```/docker-ros/ws/src/target``` inside the container. This means any changes made locally will also be reflected inside the container, and any changes made inside the container will appear locally.
- Any dependencies listed in ```dependencies.repositories``` will be added to ```/docker-ros/ws/src/upstream```. For instance, the repository vortex-msgs will be added to this directory but will not be mounted.
3. The Docker container runs with the --network host option. This enables the container to see other ROS 2 topics and nodes on the host network. However, note that this functionality is not supported on macOS.

### Troubleshooting
#### Platform Compatibility (e.g, ARM64 vs AMD64)
[docker-ros](https://github.com/ika-rwth-aachen/docker-ros) will automatically detect the platform, but if you're encountering issues with Docker due to platform incompatibilities (like arm64 vs. amd64), follow these steps:
1. Check Your Computers Architecture
2. In ```entrypoint.sh```, uncomment and adjust the PLATFORM variable:
- For AMD64:
```bash
export PLATFORM="amd64"
```
- For ARM64:
```bash
export PLATFORM="linux/arm64"
```
3. After adjusting the platform, rebuild the Docker image and run the container:
```bash
./entrypoint.sh
```

## Documentation
* TODO: Drivers and hardware specifics for each drone will be added to the wiki. Link them here.
* TODO: How to adapt the software stack to new hardware.
* A collection of master theses written by Vortex members:
  *   [Manta v1: A Deliberative Agent Software Architecture for Autonomous Underwater Vehicles](https://github.com/vortexntnu/Vortex-AUV/blob/documentation/top-level_readme/docs/master_theses/Kristoffer%20Solberg%20(2020).pdf)
  *   [A real-time DVL and pressure sensor AINS comparison study between EKF, ESKF and NLO for Manta-2020](https://github.com/vortexntnu/Vortex-AUV/blob/documentation/top-level_readme/docs/master_theses/Oyvind%20Denvik%20(2020).pdf)
  *   [Sonar EKF-SLAM and mapping inanstructured underwater environment](https://github.com/vortexntnu/Vortex-AUV/blob/documentation/top-level_readme/docs/master_theses/Ambj%C3%B8rn%20Waldum%20(2020).pdf)
  *   [Autonomous Navigation, Mapping, and Exploration for Underwater Robots](https://github.com/vortexntnu/Vortex-AUV/blob/documentation/top-level_readme/docs/master_theses/V%C3%A5ge%2C%20Utbjoe%2C%20Gjerden%20og%20Engebretsen%20(2019).pdf)
