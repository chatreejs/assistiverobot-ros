cmake_minimum_required(VERSION 3.0.2)
project(toktak_keyop)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_srvs
  std_msgs
  roscpp
  ecl_exceptions
  ecl_threads
  ecl_time
  kobuki_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES toktak_keyop
  CATKIN_DEPENDS
  geometry_msgs
  std_srvs
  std_msgs
  roscpp
  ecl_exceptions
  ecl_threads
  ecl_time
  kobuki_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_subdirectory(src)

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h*")

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
