
# Packages for Livox LiDARs for ROS1 and ROS2

## Prerequisites

- Ubuntu 18.04 for ROS Melodic

- Ubuntu 20.04 for ROS Noetic and ROS2 Foxy

- Ubuntu 22.04 for ROS2 Humble

## Alternative: automated build
The package provides a script that installs everything automatically for you. You first has to create a catkin workspace and place the _match_livox_ folder in __[your_workspace]/src/__. It should have this structure:
```
user/
└── your_workspace/
    └── src/
        └── match_livox
```
Then run the script within the _match_livox_ package, passing your ROS version as argument to it (ROS1|ROS2|humble). For example, if your ROS version is ROS1:
```
./auto_build.sh ROS1
```
In case the permission to run the script is denied, run:
```
chmod +x auto_build.sh
```
When the build is finished, the installed packages should have the following structure:
```
user/
├── Livox-SDK2
└── your_workspace/
    └── src/
        ├── livox_ros_driver2
        └── match_livox
```

## Alternative: Build from source

1. Follow the instruction for installing [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2).

2. Follow the instruction for installing [Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2) . __Make sure to clone the repository in [your_workspace]/src/__.
3. Copy the _match_livox_ folder to the same directory as _livox_ros_driver2_. Then from the root of your workspace folder, run `catkin_make`.

  
## Usage for match's mur620b robot
The initial configurations have already been done for this specific robot. To activate the LiDAR sensor, firstly source to your workspace:
```
source [your_workspace]/devel/setup.bash
``` 
Then run:
```
roslaunch match_livox msg_MID360_mur620b.launch
```

##  General guide
For any other use case, prior configurations might be necessary.

### Initial configurations

1. Find the IP address of your LiDAR: refer to the [instruction manual](https://www.livoxtech.com/downloads) of your device. For example, the default IP address of the MID-360 sensor is 192.168.1.1XX (XX stands for the last two digits of the Livox Mid-360 LiDAR sensor’s serial number).

2. Configure your host device IP address and subnet mask if needed. Make sure that it is static IP!

3. You can configure the IP addresses of the host PC and of the LiDAR device in the .json files under `livox_ros_driver2/config`. Refer to the [Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2) page for further details.

4. [Livox Viewer 2](https://www.livoxtech.com/downloads): this software can be used to configure the IP address of your LiDAR sensor. See [Livox Viewer 2 User Manual](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid360/Livox_Viewer_2_User_Manual_EN_v1.2.pdf). This is extremely useful when you don't want to change the IP address of your host device.

5. Launch script: the launch file template can be found under `livox_ros_driver2/launch_ROS1` for ROS1 and `livox_ros_driver2/launch_ROS2` for ROS2. Refer to the [Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2) page for further details on the main parameters.
