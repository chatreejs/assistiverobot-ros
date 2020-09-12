# An Assistive Robot for Convenient Delivery in Indoor Environment

[![Build Status](https://img.shields.io/gitlab/pipeline/assistiverobot/assistiverobot.ros?style=flat-square)](https://gitlab.com/assistiverobot/assistiverobot.ros/-/pipelines)
[![ROS Distro](https://img.shields.io/badge/ROS-melodic-brightgreen?style=flat-square)](https://img.shields.io/badge/ROS-melodic-brightgreen)

This is the code repository for ROS Robotics using [Kobuki](http://kobuki.yujinrobot.com/about2/) and [RPLIDAR A1](http://www.slamtec.com/en/lidar/a1) to make a delivery robot using slam and navigation, published by Chanon Treemeth and Jirawat Promsee.

## Prerequisites

- [ROS](https://www.ros.org/install/)
- [Catkin Workspace](http://wiki.ros.org/catkin/workspaces)
- [Kobuki Package](http://wiki.ros.org/kobuki)
- [Navigation Package](http://wiki.ros.org/navigation)
- [Open SLAM Package](http://wiki.ros.org/openslam_gmapping)
- [RPLIDAR Package]()

See detailed installation instructions [here](installation.md).

If you don't have catkin workspace, Let's create and build a catkin workspace:

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

## Usage

Firstly, clone this repository to your catkin workspace.

```bash
$ cd ~/catkin_ws/src
$ git clone https://gitlab.com/assistiverobot/assistiverobot.ros.git
```

Build package using `catkin_make`.

```bash
$ cd ~/catkin_ws/
$ catkin_make
```

Setup environment of your current shell.

```bash
$ source ~/catkin_ws/devel/setup.bash
```

## Real-world robot using kobuki base

### Bring up

You must start `toktak_node` before launch other scripts

```bash
$ roslaunch toktak_node minimal.launch --screen
```

### Teleoperation

To start `keyboard teleoperation` launch the **keyop.launch** file in a new **Shell**

```bash
$ roslaunch toktak_keyop keyop.launch
```

### Running OpenSLAM GMapping

To start `GMapping` launch the **toktak_gmapping.launch** file in a new **Shell**

```bash
$ roslaunch toktak_slam toktak_gmapping.launch gazebo:=false
```

To save map from slam launch the **map_saver** in a new **Shell**

```bash
$ roscd toktak_navigation/maps
$ rosrun map_server map_saver -f ./<MAP_NAME>
```

### Running Navigation

To start `navigation` launch the **toktak_navigation.launch** file in a new **Shell**

```bash
$ roslaunch toktak_navigation toktak_navigation.launch gazebo:=false map_name:=<MAP_NAME>
```

### Application

To start assitive robot delivery demo1 cancel all launch (e.g. `toktak_node`) and launch the **toktak_demo1.launch** with out `toktak_node` because this launch file already includes `toktak_node`.

```bash
$ roslaunch toktak_apps toktak_demo1.launch
```

## Simulation robot

You must spawn robot to world before launch other scripts

Launch the `gazebo simulation` and **spawn** the robot in an **empty world** or **other world**.

```bash
$ roslaunch toktak_gazebo spawn.launch world:=<WORLD_NAME>
```

if `<WORLD_NAME>` not exists in `toktak_gazebo/world/` it will become an `empty_world` automatically.

### Teleoperation

To start `keyboard teleoperation` launch the **keyop.launch** file in a new **Shell**

```bash
$ roslaunch toktak_keyop keyop.launch
```

### Running OpenSLAM GMapping

To start `GMapping` launch the **toktak_gazebo_gmapping.launch** file in a new **Shell**

```bash
$ roslaunch toktak_slam toktak_gmapping.launch
```

To save map from slam launch the **map_saver** in a new **Shell**

```bash
$ roscd toktak_navigation/maps
$ rosrun map_server map_saver -f ./<MAP_NAME>
```

### Running Navigation

To start `navigation` launch the **toktak_navigation.launch** file in a new **Shell**

```bash
$ roslaunch toktak_navigation toktak_navigation.launch map_name:=<MAP_NAME>
```

## Troubleshooting

### [gazebo-2] process has died , error exit code 255

The most probable cause is that you have the gzclient or gzserver opened. Try with

```bash
$ killall gzserver
$ kilall gzclient
```

### ERROR: cannot launch node of type [gmapping/slam_gmapping]

```
ERROR: cannot launch node of type [gmapping/slam_gmapping]: gmapping
```

It seems like gmapping is missing. Try

```bash
$ sudo apt-get install ros-<DISTRO>-gmapping
```

### CMake Error could not find a package configuration file provided by "openslam_gmapping"

If you see error like

```
CMake Error at /opt/ros/melodic/share/catkin/cmake/catkinConfig.cmake:83 (find_package):
  Could not find a package configuration file provided by "openslam_gmapping"
  with any of the following names:

    openslam_gmappingConfig.cmake
    openslam_gmapping-config.cmake

  Add the installation prefix of "openslam_gmapping" to CMAKE_PREFIX_PATH or
  set "openslam_gmapping_DIR" to a directory containing one of the above
  files.  If "openslam_gmapping" provides a separate development package or
  SDK, be sure it has been installed.
```

It seems like openslam_gmapping is missing. Try

```bash
$ sudo apt-get install ros-<DISTRO>-openslam-gmapping
```

### ERROR: cannot launch node of type [map_server/map_server]: map_server, [amcl/amcl]: amcl, [move_base/move_base]: move_base

It seems like navigation package is missing. Try

```bash
$ sudo apt-get install ros-<DISTRO>-navigation-tutorials
```

### Failed to create the dwa_local_planner/DWAPlannerROS planner

It seems like dwa-local-planner is missing. Try

```bash
$ sudo apt-get install ros-<DISTRO>-dwa-local-planner
```

## Contributors

<a href="https://gitlab.com/chatreejs" style="margin-right: .25em">
  <img src="https://gitlab.com/uploads/-/system/user/avatar/5542080/avatar.png?width=400" title="chatreejs" width="50" height="50">
</a>
<a href="https://github.com/tarrelateto1">
  <img src="https://avatars1.githubusercontent.com/u/47720165?s=460&v=4" title="tarrelateto1" width="50" height="50">
</a>

## License

MIT ©
