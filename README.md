# RoboOrchard Deploy ROS2
[![Linux platform](https://img.shields.io/badge/platform-linux--64-green.svg)](https://releases.ubuntu.com/22.04/)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-2dceef)](https://docs.ros.org/en/humble/index.html)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](https://github.com/HorizonRobotics/robo_orchard_lab/blob/master/LICENSE)

## Introduction
This project includes two ROS2 robot toolkits: [robo_orchard_calibrator](src/robo_orchard_calibrator/) and [robo_orchard_deploy](src/robo_orchard_deploy/). *robo_orchard_calibrator* could be configured to perform hand-eye calibration of different robotic arms, while *robo_orchard_deploy* could be used to efficiently deploy the manipulation model in real robot.

## Prerequisites

Before you begin, ensure you have the following installed on your system:

- Ubuntu 22.04

- ROS 2 Humble Hawksbill

- Python 3.10+

- `make` build automation tool

## Build
### Download
```bash
mkdir ros2_ws
cd ros2_ws
git clone https://github.com/HorizonRobotics/robo_orchard_deploy_ros2 .
```

### Build
We provide Makefile to simplify the setup. While we recommend using a `Python virtual environment`. You can also use colcon build diretly.
```bash
# prepare env
make dev-env
# make
make ros2_build
# adjust env
source install/setup.bash
```
After compilation, the directory structure will be structured as follows. The src directory contains ROS2 packages.
```bash
.
├── example
│   ├── config
│   └── scripts
├── scm
│   ├── lint
│   └── qac
└── src
    ├── robo_orchard_calibrator
    │   ├── resource
    │   └── robo_orchard_calibrator
    └── robo_orchard_deploy
        ├── launch
        ├── resource
        └── robo_orchard_deploy
```

## License
**robo_orchard_ros2** is open-source and licensed under the [Apache License 2.0](https://github.com/HorizonRobotics/robo_orchard_deploy_ros2/blob/master/LICENSE). If you are interested in contributing, please reach out to us.