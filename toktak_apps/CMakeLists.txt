cmake_minimum_required(VERSION 2.8.3)
project(toktak_apps)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  actionlib_msgs
  std_msgs
  rospy
  kobuki_msgs
)

catkin_package(
  LIBRARIES toktak_apps
  CATKIN_DEPENDS
  geometry_msgs
  sensor_msgs
  actionlib_msgs
  std_msgs
  rospy
  kobuki_msgs
)