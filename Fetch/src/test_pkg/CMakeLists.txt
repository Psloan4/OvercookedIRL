cmake_minimum_required(VERSION 3.0.2)
project(test_pkg)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
)

################################################
## catkin specific configuration ##
################################################
catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    rospy
    std_msgs
    sensor_msgs
    cv_bridge
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Install Python nodes
catkin_install_python(
  PROGRAMS
    src/fetch_movement.py
    src/camera_viewer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############
# (leave default testing stuff commented out)
