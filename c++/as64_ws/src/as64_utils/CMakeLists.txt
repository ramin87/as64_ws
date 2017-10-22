cmake_minimum_required(VERSION 2.8.3)

project(as64_utils)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#find_package(orocos_kdl)
find_package(catkin REQUIRED
  #orocos_kdl # use KDL functionalities
  roscpp
  std_msgs
  geometry_msgs
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

#find_package(Eigen REQUIRED)
#include_directories(${Eigen_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs
  DEPENDS opencv pcl eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3
)

############################
######     FILTER   ########
############################

add_library(as64-filter
  src/filter/movingWinMeanFilter.cpp
)

add_dependencies(as64-filter
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(as64-filter
  ${catkin_LIBRARIES}
  #orocos-kdl
)

##########################
######     MISC   ########
##########################

add_library(as64-misc
  src/misc/timer.cpp
)

add_dependencies(as64-misc
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(as64-misc
  ${catkin_LIBRARIES}
  #orocos-kdl
  #armadillo
)

##########################
######     MATH   ########
##########################

add_library(as64-math
  src/math/math.cpp
  src/math/quaternions.cpp
  src/math/convertions.cpp
)

add_dependencies(as64-math
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(as64-math
  ${catkin_LIBRARIES}
  #orocos-kdl
  #armadillo
)

