# Kobuki SLAM Navigation
[![Build Status](https://travis-ci.com/Chanonsersa/Kobuki-SLAM-Navigation.svg?branch=master)](https://travis-ci.com/Chanonsersa/Kobuki-SLAM-Navigation)

This is the code repository for ROS Robotics using [kobuki](http://kobuki.yujinrobot.com/about2/) to make a slam and navigation , published by Chanon Treemeth and Jirawat Promsee.

## Prerequisites

* [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* [Catkin Workspace](http://wiki.ros.org/catkin/workspaces)
* [Kobuki Package](http://wiki.ros.org/kobuki/Tutorials/Installation)

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

`$ source devel/setup.bash`

## Real-world robot using kobuki

Only support at ROS Kinetic

## Simulation

### 1. Simulation two wheel robot

#### Spawn robot to world

Launch the `gazebo simulation` and **spawn** the robot in an **empty world** or **other world**.
  * `$ roslaunch m2wr_description spawn.launch world:=<WORLD_NAME>`
  * if `<WORLD_NAME>` not exists in `/my_worlds/world/` it will become an `empty_world` automatically.
  
To start `rviz` visualiztion launch the **rviz.launch** file in a new **Shell** 
  * `$ roslaunch m2wr_description rviz.launch`

#### Running Obstacle Avoidance Algorithm

To start `obstacle avoidance algorithm` run the **obstacle_avoidance.py** in a new **Shell**

* `$ rosrun motion_plan obstacle_avoidance.py`

#### Running Open SLAM GMapping

To start `GMapping` launch the **gmapping.launch** file in a new **Shell**

* `$ roslaunch motion_plan gmapping.launch`

### 2. Simulation Kobuki

Only support at ROS Kinetic

## Troubleshooting
### [gazebo-2] process has died , error exit code 255
The most probable cause is that you have the gzclient or gzserver opened. Try with

```
$ killall gzserver
$ kilall gzclient
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

`sudo apt-get install ros-<DISTRO>-openslam-gmapping`


## Contributors

<a href="https://github.com/Chanonsersa"><img src="https://avatars0.githubusercontent.com/u/36321701?s=460&v=4" title="Chanonsersa" width="80" height="80"></a>

<a href="https://github.com/tarrelateto1"><img src="https://avatars1.githubusercontent.com/u/47720165?s=460&v=4" title="tarrelateto1" width="80" height="80"></a>

## License

MIT ©
