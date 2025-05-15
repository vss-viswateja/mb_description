# Visual SLAM for Autonomous Navigation using RTAB-Map

[![ROS 2](https://img.shields.io/badge/ROS-2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Webots](https://img.shields.io/badge/Webots-R2023a-blue)](https://cyberbotics.com/)
[![RTAB-Map](https://img.shields.io/badge/RTAB--Map-0.20.22-green)](https://introlab.github.io/rtabmap/)

A complete implementation of Visual SLAM using RTAB-Map with sensor fusion in Webots simulation environment, featuring ROSbot integration and real-time 3D mapping.

## Contents
- [Description](#description)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Acknowledgements](#acknowledgements)

## Description

This package implements a Visual SLAM solution for autonomous navigation using:
- **Webots** simulation environment with ROSbot
- **Robot Localization** package for EKF-based sensor fusion (IMU + wheel odometry)
- **RTAB-Map** for real-time appearance-based mapping
- **TF2** for coordinate transformations
- **Differential Drive Controller** for robot motion

Key features:
- Real-time 3D environment mapping
- Sensor fusion with Extended Kalman Filter
- Loop closure detection using Bag-of-Words
- ROS 2 integration for simulation and perception

## Prerequisites

- **ROS 2 Humble Hawksbill**
- **Webots R2023a** or newer
- **RTAB-Map ROS 2** (`ros-humble-rtabmap-ros`)
- Required ROS packages:
  ```bash
  sudo apt install ros-humble-robot-localization \
  ros-humble-tf2-ros \
  ros-humble-joint-state-publisher \
  ros-humble-depthimage-to-laserscan
