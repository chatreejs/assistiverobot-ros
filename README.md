# ROS-Robotics

This is the code repository for ROS Robotics , published by Chanon Treemeth and Jirawat Promsee.

## Prerequisites

* [ROS](http://wiki.ros.org/)
* [Catkin Workspace](http://wiki.ros.org/catkin/workspaces)

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

## Simulation

### Simulation two wheel robot with laser scan sensor

Launch the `gazebo simulation` and **spawn** the robot in an **empty world**.

`$ roslaunch m2wr_description spawn.launch`

To start `rviz` visualiztion launch the **rviz.launch** file in a new **Shell** 

`$ roslaunch m2wr_description rviz.launch`

### Running Obstacle Avoidance Algorithm

Launch the `gazebo simulation` and **spawn** the robot in an **18 floor world**.

`$ roslaunch m2wr_description 18_floor.launch`

To start `obstacle avoidance algorithm` run the **obstacle_avoidance.py** in a new **Shell**

`$ rosrun motion_plan obstacle_avoidance.py`

## Troubleshooting
### [gazebo-2] process has died , exit code 255
The most probable cause is that you have the gzclient or gzserver opened. Try with 
```
$ killall gzserver
$ kilall gzclient
```

