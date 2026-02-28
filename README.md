# piper_ros

This repository provides a set of ROS 2 packages to integrate [piper](https://github.com/OHF-Voice/piper1-gpl) TTS (Text-to-Speech) into ROS 2 using [audio_common](https://github.com/mgonzs13/audio_common).

<div align="center">

[![License: MIT](https://img.shields.io/badge/GitHub-MIT-informational)](https://opensource.org/license/mit) [![GitHub release](https://img.shields.io/github/release/mgonzs13/piper_ros.svg)](https://github.com/mgonzs13/piper_ros/releases) [![Code Size](https://img.shields.io/github/languages/code-size/mgonzs13/piper_ros.svg?branch=main)](https://github.com/mgonzs13/piper_ros?branch=main) [![Last Commit](https://img.shields.io/github/last-commit/mgonzs13/piper_ros.svg)](https://github.com/mgonzs13/piper_ros/commits/main) [![GitHub issues](https://img.shields.io/github/issues/mgonzs13/piper_ros)](https://github.com/mgonzs13/piper_ros/issues) [![GitHub pull requests](https://img.shields.io/github/issues-pr/mgonzs13/piper_ros)](https://github.com/mgonzs13/piper_ros/pulls) [![Contributors](https://img.shields.io/github/contributors/mgonzs13/piper_ros.svg)](https://github.com/mgonzs13/piper_ros/graphs/contributors) [![Python Formatter Check](https://github.com/mgonzs13/piper_ros/actions/workflows/python-formatter.yml/badge.svg?branch=main)](https://github.com/mgonzs13/piper_ros/actions/workflows/python-formatter.yml?branch=main) [![C++ Formatter Check](https://github.com/mgonzs13/piper_ros/actions/workflows/cpp-formatter.yml/badge.svg?branch=main)](https://github.com/mgonzs13/piper_ros/actions/workflows/cpp-formatter.yml?branch=main) [![Doxygen Deployment](https://github.com/mgonzs13/piper_ros/actions/workflows/doxygen-deployment.yml/badge.svg)](https://mgonzs13.github.io/piper_ros/latest)

| ROS 2 Distro |                          Branch                           |                                                                                                     Build status                                                                                                     |                                                                Docker Image                                                                |
| :----------: | :-------------------------------------------------------: | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------: | :----------------------------------------------------------------------------------------------------------------------------------------: |
|  **Humble**  | [`main`](https://github.com/mgonzs13/piper_ros/tree/main) |  [![Humble Build](https://github.com/mgonzs13/piper_ros/actions/workflows/humble-build-test.yml/badge.svg?branch=main)](https://github.com/mgonzs13/piper_ros/actions/workflows/humble-build-test.yml?branch=main)   |  [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-humble-blue)](https://hub.docker.com/r/mgons/piper_ros/tags?name=humble)  |
|   **Iron**   | [`main`](https://github.com/mgonzs13/piper_ros/tree/main) |     [![Iron Build](https://github.com/mgonzs13/piper_ros/actions/workflows/iron-build-test.yml/badge.svg?branch=main)](https://github.com/mgonzs13/piper_ros/actions/workflows/iron-build-test.yml?branch=main)      |    [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-iron-blue)](https://hub.docker.com/r/mgons/piper_ros/tags?name=iron)    |
|  **Jazzy**   | [`main`](https://github.com/mgonzs13/piper_ros/tree/main) |    [![Jazzy Build](https://github.com/mgonzs13/piper_ros/actions/workflows/jazzy-build-test.yml/badge.svg?branch=main)](https://github.com/mgonzs13/piper_ros/actions/workflows/jazzy-build-test.yml?branch=main)    |   [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-jazzy-blue)](https://hub.docker.com/r/mgons/piper_ros/tags?name=jazzy)   |
|  **Kilted**  | [`main`](https://github.com/mgonzs13/piper_ros/tree/main) |  [![Kilted Build](https://github.com/mgonzs13/piper_ros/actions/workflows/kilted-build-test.yml/badge.svg?branch=main)](https://github.com/mgonzs13/piper_ros/actions/workflows/kilted-build-test.yml?branch=main)   |  [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-kilted-blue)](https://hub.docker.com/r/mgons/piper_ros/tags?name=kilted)  |
| **Rolling**  | [`main`](https://github.com/mgonzs13/piper_ros/tree/main) | [![Rolling Build](https://github.com/mgonzs13/piper_ros/actions/workflows/rolling-build-test.yml/badge.svg?branch=main)](https://github.com/mgonzs13/piper_ros/actions/workflows/rolling-build-test.yml?branch=main) | [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-rolling-blue)](https://hub.docker.com/r/mgons/piper_ros/tags?name=rolling) |

</div>

## Table of Contents

1. [Installation](#installation)
2. [Docker](#docker)
3. [Usage](#usage)
4. [Params](#params)

## Installation

To run piper_ros follow the next commands:

```shell
cd ~/ros2_ws/src
git clone https://github.com/mgonzs13/audio_common.git
git clone https://github.com/mgonzs13/piper_ros.git
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
docker run -it --rm --device /dev/snd piper_ros
```

## Usage

```shell
ros2 launch piper_bringup piper.launch.py
```

```shell
ros2 action send_goal /say audio_common_msgs/action/TTS "{'text': 'Hello World from ros 2'}"
```

### Spanish Example

```shell
ros2 launch piper_bringup piper_spanish.launch.py
```

```shell
ros2 action send_goal /say audio_common_msgs/action/TTS "{'text': 'Hola Mundo desde ros 2'}"
```

## Params

### General Parameters

| Param      | Type     | Default | Description                                          |
| ---------- | -------- | ------- | ---------------------------------------------------- |
| `chunk`    | `int32`  | `512`   | Chunk size in samples for audio publication.         |
| `frame_id` | `string` | `""`    | Frame ID attached to published AudioStamped headers. |

### Model Parameters (`model.*`)

| Param                   | Type     | Default                                            | Description                                                                                            |
| ----------------------- | -------- | -------------------------------------------------- | ------------------------------------------------------------------------------------------------------ |
| `model.repo`            | `string` | `"rhasspy/piper-voices"`                           | HuggingFace repository for model download.                                                             |
| `model.filename`        | `string` | `"en/en_US/lessac/low/en_US-lessac-low.onnx"`      | Filename of the model in the repository.                                                               |
| `model.path`            | `string` | `""`                                               | Local path to the voice model file. If empty, the model is downloaded from `model.repo`.               |
| `model.config_repo`     | `string` | `"rhasspy/piper-voices"`                           | HuggingFace repository for the model config download.                                                  |
| `model.config_filename` | `string` | `"en/en_US/lessac/low/en_US-lessac-low.onnx.json"` | Filename of the model config in the repository.                                                        |
| `model.config_path`     | `string` | `""`                                               | Local path to the JSON voice config file. If empty, the config is downloaded from `model.config_repo`. |

### Synthesis Parameters (`synthesis.*`)

| Param                                | Type    | Default | Description                                             |
| ------------------------------------ | ------- | ------- | ------------------------------------------------------- |
| `synthesis.speaker_id`               | `int32` | `0`     | Numerical speaker ID for multi-speaker voices.          |
| `synthesis.noise_scale`              | `float` | `0.667` | Amount of noise added during audio generation.          |
| `synthesis.length_scale`             | `float` | `1.0`   | Speed of speaking (1 = normal, < 1 faster, > 1 slower). |
| `synthesis.noise_w_scale`            | `float` | `0.8`   | Variation in phoneme lengths during synthesis.          |
| `synthesis.sentence_silence_seconds` | `float` | `0.2`   | Seconds of silence inserted between sentences.          |
