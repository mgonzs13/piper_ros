# piper_ros

This repository provides a set of ROS 2 packages to integrate [piper](https://github.com/rhasspy/piper) into ROS 2 using [audio_common](https://github.com/mgonzs13/audio_common) [4.0.3](https://github.com/mgonzs13/audio_common/releases/tag/4.0.3).

[![License: MIT](https://img.shields.io/badge/GitHub-MIT-informational)](https://opensource.org/license/mit) [![GitHub release](https://img.shields.io/github/release/mgonzs13/piper_ros.svg)](https://github.com/mgonzs13/piper_ros/releases) [![Code Size](https://img.shields.io/github/languages/code-size/mgonzs13/piper_ros.svg?branch=main)](https://github.com/mgonzs13/piper_ros?branch=main) [![Last Commit](https://img.shields.io/github/last-commit/mgonzs13/piper_ros.svg)](https://github.com/mgonzs13/piper_ros/commits/main) [![GitHub issues](https://img.shields.io/github/issues/mgonzs13/piper_ros)](https://github.com/mgonzs13/piper_ros/issues) [![GitHub pull requests](https://img.shields.io/github/issues-pr/mgonzs13/piper_ros)](https://github.com/mgonzs13/piper_ros/pulls) [![Contributors](https://img.shields.io/github/contributors/mgonzs13/piper_ros.svg)](https://github.com/mgonzs13/piper_ros/graphs/contributors) [![Python Formatter Check](https://github.com/mgonzs13/piper_ros/actions/workflows/python-formatter.yml/badge.svg?branch=main)](https://github.com/mgonzs13/piper_ros/actions/workflows/python-formatter.yml?branch=main) [![C++ Formatter Check](https://github.com/mgonzs13/piper_ros/actions/workflows/cpp-formatter.yml/badge.svg?branch=main)](https://github.com/mgonzs13/piper_ros/actions/workflows/cpp-formatter.yml?branch=main)

<div align="center">

| ROS 2 Distro |                          Branch                           |                                                                                                       Build status                                                                                                       |                                                                Docker Image                                                                | Documentation                                                                                                                                            |
| :----------: | :-------------------------------------------------------: | :----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------: | :----------------------------------------------------------------------------------------------------------------------------------------: | -------------------------------------------------------------------------------------------------------------------------------------------------------- |
|  **Humble**  | [`main`](https://github.com/mgonzs13/piper_ros/tree/main) |  [![Humble Build](https://github.com/mgonzs13/piper_ros/actions/workflows/humble-docker-build.yml/badge.svg?branch=main)](https://github.com/mgonzs13/piper_ros/actions/workflows/humble-docker-build.yml?branch=main)   |  [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-humble-blue)](https://hub.docker.com/r/mgons/piper_ros/tags?name=humble)  | [![Doxygen Deployment](https://github.com/mgonzs13/piper_ros/actions/workflows/doxygen-deployment.yml/badge.svg)](https://mgonzs13.github.io/piper_ros/) |
|   **Iron**   | [`main`](https://github.com/mgonzs13/piper_ros/tree/main) |     [![Iron Build](https://github.com/mgonzs13/piper_ros/actions/workflows/iron-docker-build.yml/badge.svg?branch=main)](https://github.com/mgonzs13/piper_ros/actions/workflows/iron-docker-build.yml?branch=main)      |    [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-iron-blue)](https://hub.docker.com/r/mgons/piper_ros/tags?name=iron)    | [![Doxygen Deployment](https://github.com/mgonzs13/piper_ros/actions/workflows/doxygen-deployment.yml/badge.svg)](https://mgonzs13.github.io/piper_ros/) |
|  **Jazzy**   | [`main`](https://github.com/mgonzs13/piper_ros/tree/main) |    [![Jazzy Build](https://github.com/mgonzs13/piper_ros/actions/workflows/jazzy-docker-build.yml/badge.svg?branch=main)](https://github.com/mgonzs13/piper_ros/actions/workflows/jazzy-docker-build.yml?branch=main)    |   [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-jazzy-blue)](https://hub.docker.com/r/mgons/piper_ros/tags?name=jazzy)   | [![Doxygen Deployment](https://github.com/mgonzs13/piper_ros/actions/workflows/doxygen-deployment.yml/badge.svg)](https://mgonzs13.github.io/piper_ros/) |
| **Rolling**  | [`main`](https://github.com/mgonzs13/piper_ros/tree/main) | [![Rolling Build](https://github.com/mgonzs13/piper_ros/actions/workflows/rolling-docker-build.yml/badge.svg?branch=main)](https://github.com/mgonzs13/piper_ros/actions/workflows/rolling-docker-build.yml?branch=main) | [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-rolling-blue)](https://hub.docker.com/r/mgons/piper_ros/tags?name=rolling) | [![Doxygen Deployment](https://github.com/mgonzs13/piper_ros/actions/workflows/doxygen-deployment.yml/badge.svg)](https://mgonzs13.github.io/piper_ros/) |

</div>

## Table of Contents

1. [Installation](#installation)
2. [Docker](#docker)
3. [Usage](#usage)

## Installation

To run piper_ros follow the next commmands:

```shell
cd ~/ros2_ws/src
git clone https://github.com/mgonzs13/audio_common.git
git clone https://github.com/mgonzs13/piper_ros.git
pip3 install -r piper_ros/requirements.txt
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## Docker

You can build the piper_ros docker:

```shell
docker build -t piper_ros .
```

Then, you can run the docker container:

```shell
docker run -it --rm piper_ros
```

## Usage

```shell
ros2 launch piper_bringup piper.launch.py
```

```shell
ros2 action send_goal /say audio_common_msgs/action/TTS "{'text': 'Hello World'}
```
