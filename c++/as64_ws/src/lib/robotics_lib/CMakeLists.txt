cmake_minimum_required(VERSION 2.8.3)

project(robotics_lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#set(CMAKE_POSITION_INDEPENDENT_CODE ON)

#find_package(orocos_kdl)
find_package(catkin REQUIRED
  #orocos_kdl # use KDL functionalities
  roscpp
  roslib
  autharl_core
  lwr_robot
  io_lib
  time_lib
)

find_package(Yaml-cpp REQUIRED)
#find_package(OpenCV REQUIRED)
#find_package(PCL REQUIRED)

#find_package(Eigen REQUIRED)
#include_directories(${Eigen_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp #std_msgs geometry_msgs
#  DEPENDS opencv pcl eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3
)

##############################
######     Robotics   ########
##############################

add_library(robotics-lib
  src/robotics.cpp
  src/jlav.cpp
  src/control/GravityCompController.cpp
)

add_dependencies(robotics-lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(robotics-lib
  ${catkin_LIBRARIES}
  armadillo
  autharl-robot
  autharl-viz
  lwr-robot
  pthread
  IO-lib
  time-lib
  yaml-cpp
  #orocos-kdl
)
