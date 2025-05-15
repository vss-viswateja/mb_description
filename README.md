# Visual SLAM for Autonomous Navigation using RTAB-Map

[![ROS](https://img.shields.io/badge/ROS-2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Webots](https://img.shields.io/badge/Webots-R2023a-blue)](https://cyberbotics.com/)
[![RTAB-Map](https://img.shields.io/badge/RTAB--Map-0.20.22-green)](https://introlab.github.io/rtabmap/)

A complete implementation of Visual SLAM using RTAB-Map with sensor fusion in the Webots simulation environment, featuring ROSbot integration and real-time 3D mapping.

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

Before running this package, ensure you have the following software installed:

-   **Webots Simulator:** Install Webots from the official Cyberbotics website: [Webots](https://cyberbotics.com/) It is compatible with ROS 2.
-   **ROS 2 (Galactic/Humble or later):** Follow the official ROS 2 installation instructions for your operating system. You can find the documentation here: [ros2 documentation](https://docs.ros.org/en/rolling/index.html) (Replace "rolling" with your ROS 2 distribution, e.g., "galactic", "humble")
-   **webots\_ros2:** This package provides the necessary interface between ROS 2 and Webots, enabling communication and control of robots within the simulator. You can install the e-puck specific portion of it using:

    ```bash
    sudo apt install ros-<ros2-distro>-webots-ros2
    ```

    Replace `<ros2-distro>` with your ROS 2 distribution (e.g., `galactic`, `humble`).
-   **Colcon (ROS 2 build tool):** If you don't have it, install it:

    ```bash
    sudo apt install python3-colcon-common-extensions
    ```
-   **R-TAB Map:** This is the V-SLAM package used in this project :

    ```bash
    sudo apt install ros-<ros2-distro>-rtabmap-ros
    ```
## Installation

-  **Clone the repository:**

    Navigate to your ROS 2 workspace's `src` directory (or wherever you manage your ROS 2 packages). Then, clone the repository using Git:

    ```bash
    cd <your_ros2_workspace>/src
    git clone <repository_url>
    ```

    Replace `<your_ros2_workspace>` with the path to your ROS 2 workspace (e.g., `~/ros2_ws`) and `<repository_url>` with the URL of this GitHub repository.

-  **Install Dependencies:**

    Navigate back to your ROS 2 workspace's root directory:

    ```bash
    cd <your_ros2_workspace>
    ```

    If your package has any dependencies specified in the `package.xml` file, you can install them using `rosdep`:

    ```bash
    rosdep update
    rosdep install --from-paths src --ignore-src -y
    ```

    This command will check for any missing dependencies and install them.

-  **Build the package:**

    Use `colcon` to build the package:

    ```bash
    colcon build --symlink-install
    ```

    The `--symlink-install` option creates symbolic links instead of copying files, which speeds up the build process and makes it easier to modify code.

-  **Source the setup file:**

    After building the package, you need to source the setup file to make the ROS 2 executables and libraries available in your current terminal session:

    ```bash
    source install/setup.bash
    ```

    If you want to permanently add the setup file to your environment, you can add the sourcing command to your `.bashrc` or `.zshrc` file:

    ```bash
    echo "source <your_ros2_workspace>/install/setup.bash" >> ~/.bashrc  # For bash
    # or
    echo "source <your_ros2_workspace>/install/setup.bash" >> ~/.zshrc # For zsh
    source ~/.bashrc #or source ~/.zshrc to apply the change to the current terminal
    ```

    Replace `<your_ros2_workspace>` with the actual path to your ROS 2 workspace.


    ## Usage

-  **Launch Webots with the ROSbot in simulation:**

    Open a new terminal and run the following command to launch the ROSbot simulation in Webots:

    ```bash
    ros2 launch mb_description rviz.launch.py
    ```

    This command will start the Webots simulator with the ROSbot in your defined world.

-  **Run the Visual SLAM algorithm node:**

    Open a new terminal and run the Visual SLAM node:

    ```bash
    ros2 launch rtabmap_launch rtabmap.launch.py \  rgb_topic:=/rosbot/camera_rgb/image_color \   depth_topic:=/rosbot/camera_depth/image \   camera_info_topic:=/rosbot/camera_rgb/camera_info \  frame_id:=base_link \   map_frame_id:=map \  approx_sync:=false \  args:="-d --delete_db_on_start"  rviz:=true 
    ```


-  **(Optional) Visualize the robot's path and sensor data:**

    You can use `rqt` or `rviz2` to visualize the robot's odometry, camera readings, and the generated map.

    To start `rviz`, you can run:

    ```bash
    rviz2
    ```

    Then add the desired plugins, such as "image" for camera data or "map" for a generated map.

    Then, configure the plugins to visualize the desired topics.


   ## Contributing and Bug Reporting

If you encounter any bugs, issues, or have suggestions for improvements, please feel free to contribute! You can:

* **Submit a Pull Request:** If you have code changes, bug fixes, or new features, please submit a pull request.
* **Report an Issue:** If you find a bug or have a suggestion, please open an issue on the GitHub repository. Provide as much detail as possible, including steps to reproduce the issue and any relevant error messages. 

   





