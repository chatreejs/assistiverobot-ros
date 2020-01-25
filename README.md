# Kobuki SLAM Navigation
[![Build Status](https://travis-ci.com/Chanonsersa/Kobuki-SLAM-Navigation.svg?branch=master)](https://travis-ci.com/Chanonsersa/Kobuki-SLAM-Navigation)

This is the code repository for ROS Robotics using [kobuki](http://kobuki.yujinrobot.com/about2/) to make a delivery robot using slam and navigation, published by Chanon Treemeth and Jirawat Promsee.

## Prerequisites

* [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* [Catkin Workspace](http://wiki.ros.org/catkin/workspaces)
* [Kobuki Package](http://wiki.ros.org/kobuki/Tutorials/Installation)
* [Navigation Package]()
* [Open SLAM Package]()

If you don't have catkin workspace, Let's create and build a catkin workspace:

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

## Usage

Firstly, clone this repository to your catkin workspace.

```
$ cd ~/catkin_ws/src
$ git clone https://github.com/Chanonsersa/ROS-Robotics.git
```

Build package using `catkin_make`.

```
$ cd ~/catkin_ws/
$ catkin_make
```

Setup environment of your current shell.

```$ source devel/setup.bash```

## Real-world robot using kobuki

Only support at ROS Kinetic

### Setup before using kobuki
**1. Install Kobuki**

```
$ sudo apt-get install ros-kinetic-kobuki ros-kinetic-kobuki-core
```

**2. Dialout Group**

If not already in the dialout group: 
```
$ sudo usermod -a -G dialout $USER
```
and then log out, log back in again. 

**3. Set Udev Rule**
```
$ rosrun kobuki_ftdi create_udev_rules
```
Reinsert the Kobuki's USB cable into your laptop/pc/... You should now find it show up at /dev/kobuki. 

### Bring up

You must start `kobuki_node` before launch other scripts

```
$ . /opt/ros/kinetic/setup.bash
$ roslaunch kobuki_node minimal.launch --screen
```

## Simulation Toktak (Kobuki base)

Only support at ROS Kinetic

You must spawn robot to world before launch other scripts

Launch the `gazebo simulation` and **spawn** the robot in an **empty world** or **other world**.
* `$ roslaunch toktak_description spawn.launch world:=<WORLD_NAME>`
* if `<WORLD_NAME>` not exists in `toktak_gazebo/world/` it will become an `empty_world` automatically.

### Vizualization robot

To start `rviz` visualiztion launch the **rviz.launch** file in a new **Shell** 
* `$ roslaunch toktak_description view_model.launch`

### Running Open SLAM GMapping

To start `GMapping` launch the **gmapping.launch** file in a new **Shell**

* `$ roslaunch toktak_motion_plan gmapping.launch`

### Running Navigation

To start `navigation` launch the **toktak_navigation.launch** file in a new **Shell**

* `$ roslaunch toktak_navigation toktak_navigation.launch`

## Troubleshooting
### [gazebo-2] process has died , error exit code 255
The most probable cause is that you have the gzclient or gzserver opened. Try with

```
$ killall gzserver
$ kilall gzclient
```

### ERROR: cannot launch node of type [gmapping/slam_gmapping]
If you see error like

```
ERROR: cannot launch node of type [gmapping/slam_gmapping]: gmapping
ROS path [0]=/opt/ros/kinetic/share/ros
ROS path [1]=/home/peng/turtlebot3_ws/src
ROS path [2]=/home/peng/catkin_ws/src
ROS path [3]=/home/peng/study_ws/src
ROS path [4]=/opt/ros/kinetic/share
```

It seems like gmapping is missing. Try

```sudo apt-get install ros-<DISTRO>-gmapping```

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

```
$ sudo apt-get install ros-<DISTRO>-openslam-gmapping
```

### ERROR: cannot launch node of type [map_server/map_server]: map_server, [amcl/amcl]: amcl, [move_base/move_base]: move_base

It seems like navigation package is missing. Try

```
$ sudo apt-get install ros-<DISTRO>-navigation-tutorials 
```

### Failed to create the dwa_local_planner/DWAPlannerROS planner

It seems like dwa-local-planner is missing. Try

```
$ sudo apt-get install ros-<DISTRO>-dwa-local-planner
```

## Contributors

<a href="https://github.com/Chanonsersa"><img src="https://avatars0.githubusercontent.com/u/36321701?s=460&v=4" title="Chanonsersa" width="80" height="80"></a>

<a href="https://github.com/tarrelateto1"><img src="https://avatars1.githubusercontent.com/u/47720165?s=460&v=4" title="tarrelateto1" width="80" height="80"></a>

## License

MIT ©
