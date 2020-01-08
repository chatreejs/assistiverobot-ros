# ROS-Robotics

This is the code repository for ROS Robotics , published by Chanon Treemeth and Jirawat Promsee.

# Usage

Clone this repository.

`$ git clone https://github.com/Chanonsersa/ROS-Robotics.git`

Go into project directory.

`$ cd ROS-Robotics/`

Build package using `catkin_make`.

`$ catkin_make`

Setup environment of your current shell.

`$ source devel/setup.bash`

# Simulation

## Simulation two wheel robot with laser scan sensor

Launch the `gazebo simulation` and **spawn** the robot in an **empty world**.

`$ roslaunch m2wr_description spawn.launch`

To start `rviz` visualiztion launch the **rviz.launch** file in a new **Shell** 

`$ roslaunch m2wr_description rviz.launch`

## Running Obstacle Avoidance Algorithm

Launch the `gazebo simulation` and **spawn** the robot in an **18 floor world**.

`$ roslaunch m2wr_description 18_floor.launch`

To start `obstacle avoidance algorithm` run the **obstacle_avoidance.py** in a new **Shell**

`$ rosrun motion_plan obstacle_avoidance.py`

# Troubleshooting

