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

## 1. Simulation 2 wheel robot with laser scan sensor

Launch the `gazebo simulation` and **spawn** the robot in an **empty world**.

`$ roslaunch m2wr_description spawn.launch`

To start `rviz` visualiztion launch the **rviz.launch** file in a new **Shell** 

`$ roslaunch m2wr_description rviz.launch`

After starting `rviz` you need to do the following settings
* Select **link_chasis** in the **Fixed Frame** field
* Add two new display using the **Add** button on the left bottom of rviz screen. The first display should be **RobotModel** and the other should be **LaserScan**
* Expand the **LaserScan** display by duuble click on its name and choose **Topic** as **/m2wr/laser/scan**

# Troubleshooting

