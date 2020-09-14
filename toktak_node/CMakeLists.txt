cmake_minimum_required(VERSION 2.8.3)
project(toktak_node)

find_package(catkin REQUIRED COMPONENTS
    rospy roscpp nodelet pluginlib tf angles
    geometry_msgs sensor_msgs nav_msgs std_msgs diagnostic_updater diagnostic_msgs
    kobuki_msgs kobuki_driver kobuki_keyop kobuki_safety_controller
    ecl_exceptions ecl_sigslots ecl_streams ecl_threads rplidar_ros
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES kobuki_ros kobuki_nodelet
    CATKIN_DEPENDS 
    rospy roscpp nodelet pluginlib tf angles
    geometry_msgs sensor_msgs nav_msgs std_msgs diagnostic_updater diagnostic_msgs
    kobuki_msgs kobuki_driver kobuki_keyop kobuki_safety_controller
    ecl_exceptions ecl_sigslots ecl_streams ecl_threads rplidar_ros
)