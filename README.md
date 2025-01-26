## Vortex AUV
[![Website shields.io](https://img.shields.io/website-up-down-green-red/http/shields.io.svg)](http://vortexntnu.no)
[![version](https://img.shields.io/badge/version-1.0.0-blue)](https://GitHub.com/vortexntnu/manta-auv/releases/)
![ViewCount](https://views.whatilearened.today/views/github/vortexntnu/manta-auv.svg)
[![GitHub contributors](https://img.shields.io/github/contributors/vortexntnu/manta-auv.svg)](https://GitHub.com/vortexntnu/manta-auv/graphs/contributors/)
[![Maintenance](https://img.shields.io/badge/Maintained%3F-yes-green.svg)](https://GitHub.com/vortexntnu/manta-auv/graphs/commit-activity)
[![GitHub pull-requests](https://img.shields.io/github/issues-pr/vortexntnu/manta-auv.svg)](https://GitHub.com/vortexntnu/manta-auv/pulls)
[![GitHub pull-requests closed](https://img.shields.io/github/issues-pr-closed/vortexntnu/manta-auv.svg)](https://GitHub.com/vortexntnu/manta-auv/pulls)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

![Banner](docs/banner_image.png)

This repo contains software for operating UUVs, developed by students at NTNU. The software is based on the ROS2 Humble framework, and aims to be hardware independent. Although the main focus of Vortex is autonomous operation, this software stack supports both AUV and ROV operations.

## Docker
This project uses the [docker-ros](https://github.com/ika-rwth-aachen/docker-ros) repository for building and managing Docker images. The docker-ros repository is included as a Git submodule and is configured to build images locally.

### Prerequisites
1. Docker must be installed on your system. Follow the instructions [here](https://docs.docker.com/get-docker/) to install Docker.

### Cloning
1. To clone this repository with the docker-ros submodule, use the following command:
```bash
git clone --recurse-submodules https://github.com/vortexntnu/vortex-auv.git
```
Alternatively, if you have already cloned the repository, run the following command to initialize the submodule:
```bash
git submodule update --init --recursive
```
2. Once added, docker-ros provide a script to build Docker images locally.
### Building and Running
1. Run the following command to build the Docker image:
```bash
./entrypoint.sh
```
Or, you can pull the appropriate image from [Github packages](https://github.com/vortexntnu/vortex-auv/pkgs/container/vortex-auv)

2. Once the image is built, run the following command to start the container:
```bash
docker run -it --privileged --network host --ipc=host "auv-image:latest"
```
NOTE: If running on MacOS, the `--network host` flag is not supported.

## Documentation
* TODO: Drivers and hardware specifics for each drone will be added to the wiki. Link them here.
* TODO: How to adapt the software stack to new hardware.
* A collection of master theses written by Vortex members:
  *   [Manta v1: A Deliberative Agent Software Architecture for Autonomous Underwater Vehicles](https://github.com/vortexntnu/Vortex-AUV/blob/documentation/top-level_readme/docs/master_theses/Kristoffer%20Solberg%20(2020).pdf)
  *   [A real-time DVL and pressure sensor AINS comparison study between EKF, ESKF and NLO for Manta-2020](https://github.com/vortexntnu/Vortex-AUV/blob/documentation/top-level_readme/docs/master_theses/Oyvind%20Denvik%20(2020).pdf)
  *   [Sonar EKF-SLAM and mapping inanstructured underwater environment](https://github.com/vortexntnu/Vortex-AUV/blob/documentation/top-level_readme/docs/master_theses/Ambj%C3%B8rn%20Waldum%20(2020).pdf)
  *   [Autonomous Navigation, Mapping, and Exploration for Underwater Robots](https://github.com/vortexntnu/Vortex-AUV/blob/documentation/top-level_readme/docs/master_theses/V%C3%A5ge%2C%20Utbjoe%2C%20Gjerden%20og%20Engebretsen%20(2019).pdf)
