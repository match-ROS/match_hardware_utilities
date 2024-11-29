# Packages for Livox LiDARs for ROS1 and ROS2
## Prerequisites
-   Ubuntu 18.04 for ROS Melodic
-   Ubuntu 20.04 for ROS Noetic and ROS2 Foxy
-   Ubuntu 22.04 for ROS2 Humble
## Installation
1. Follow the instruction for installing [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2).
2. Follow the instruction for installing [Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2) (be mindful about the package structure as in instructions!!!).

## Initial configurations
1. Find the IP address of your LiDAR: refer to the [instruction manual](https://www.livoxtech.com/downloads) of your device. For example, the default IP address of the MID-360 sensor is 192.168.1.1XX (XX stands for the last two digits of the Livox Mid-360 LiDAR sensor’s serial number).
2. Configure your host device IP address and subnet mask if needed. Make sure that it is static IP!
3. You can configure the IP addresses of the host PC and of the LiDAR device in the .json files under `livox_ros_driver2/config`. Refer to the [Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2) page for further details.
4. [Livox Viewer 2](https://www.livoxtech.com/downloads): this software can be used to configure the IP address of your LiDAR sensor. See [Livox Viewer 2 User Manual](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid360/Livox_Viewer_2_User_Manual_EN_v1.2.pdf). This is extremely useful when you don't want to change the IP address of your host device. 
5. Launch script: the sample launch files can be found under `livox_ros_driver2/launch_ROS1` for ROS1 and `livox_ros_driver2/launch_ROS2` for ROS2. Refer to the [Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2) page for further details on the main parameters.
## Usage
1. Source to your livox_ros_driver2 workspace:
```
source [path_to_your_ws]/devel/setup.bash
```
2. Run Livox ROS Driver 2. An example for ROS1:
```
roslaunch livox_ros_driver2 [your_launch_file]
```
For ROS2:
```
ros2 launch livox_ros_driver2 [launch file]
```
