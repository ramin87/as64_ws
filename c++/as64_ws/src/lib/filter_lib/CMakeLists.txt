cmake_minimum_required(VERSION 2.8.3)

project(filter_lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#find_package(orocos_kdl)
find_package(catkin REQUIRED
  #orocos_kdl # use KDL functionalities
  roscpp
  # OpenCV REQUIRED
  # PCL REQUIRED
  # Eigen REQUIRED
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp #std_msgs geometry_msgs
#  DEPENDS opencv pcl eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3
  # ${Eigen_INCLUDE_DIRS}
)

############################
######     FILTER   ########
############################

add_library(filter-lib
  src/filter.cpp
  src/movingWinMeanFilter.cpp
  src/svf.cpp
)

add_dependencies(filter-lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(filter-lib
  ${catkin_LIBRARIES}
  #orocos-kdl
)
