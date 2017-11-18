cmake_minimum_required(VERSION 2.8.3)

project(time_lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#find_package(orocos_kdl)
find_package(catkin REQUIRED
  #orocos_kdl # use KDL functionalities
  roscpp
)

#find_package(OpenCV REQUIRED)
#find_package(PCL REQUIRED)

#find_package(Eigen REQUIRED)
#include_directories(${Eigen_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp #std_msgs geometry_msgs
  #DEPENDS opencv pcl eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3
)

##########################
######     TIME   ########
##########################

add_library(time-lib
  src/time.cpp
  src/timer.cpp
)

add_dependencies(time-lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(time-lib
  ${catkin_LIBRARIES}
  #orocos-kdl
  #armadillo
)
