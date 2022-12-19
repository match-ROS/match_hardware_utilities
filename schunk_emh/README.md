<p align="center">
  <a href="" rel="noopener">
 <img width=200px height=200px src="https://i.imgur.com/6wj0hh6.jpg" alt="Project logo"></a>
</p>

<h3 align="center">Schunk EMH Gripper</h3>

<div align="center">

[![Status](https://img.shields.io/badge/status-active-success.svg)]()
[![GitHub Issues](https://img.shields.io/github/issues/pumablattlaus/match_hardware_utilities.svg)](https://github.com/pumablattlaus/match_hardware_utilities/issues)
[![GitHub Pull Requests](https://img.shields.io/github/issues-pr/pumablattlaus/match_hardware_utilities.svg)](https://github.com/pumablattlaus/match_hardware_utilities/pulls)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](/LICENSE)

</div>

---

<p align="center"> Class to use EMH gripper with ur
    <br> 
</p>

## 📝 Table of Contents

- [About](#about)
- [Getting Started](#getting_started)
- [Problem solving](#problems)
- [Usage](#usage)
- [TODO](./TODO.md)

## 🧐 About <a name = "about"></a>

Optimization for murs with impedance control in regard to cartesian ur-path

## 🏁 Getting Started <a name = "getting_started"></a>

1. clone repo in catkin folder and build
2. "rosrun schunk_emh schunk_controller.py" or include class in file

### Prerequisites
- rospy
- ur_ros_driver (ur_hardware_interface)

## 🔧 Problem solving <a name = "problems"></a>

### Didn't catch if gripped/released:
- change timing to wait for signal   

## 🎈 Usage <a name="usage"></a>
```
rosservice call mur/ur/schunk/grasp "{}"
rosservice call mur/ur/schunk/release "{}"
rosservice call mur/ur/schunk/is_picked "{}"
```