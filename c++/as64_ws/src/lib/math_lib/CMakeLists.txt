cmake_minimum_required(VERSION 2.8.3)

project(math_lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#find_package(orocos_kdl)
find_package(catkin REQUIRED
  #orocos_kdl # use KDL functionalities
  roscpp
  geometry_msgs
)

#find_package(OpenCV REQUIRED)
#find_package(PCL REQUIRED)

#find_package(Eigen REQUIRED)
#include_directories(${Eigen_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs # std_msgs
  #DEPENDS opencv pcl eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3
)

##########################
######     MATH   ########
##########################

add_library(math-lib
  src/math.cpp
  src/quaternions.cpp
  src/convertions.cpp
)

add_dependencies(math-lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(math-lib
  ${catkin_LIBRARIES}
  #orocos-kdl
  armadillo
)
