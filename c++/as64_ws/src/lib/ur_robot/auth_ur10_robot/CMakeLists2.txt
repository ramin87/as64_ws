cmake_minimum_required(VERSION 2.8.3)
project(ur10_robot)

# Use C++  for initialise an std::vector without a constructor
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Set the path containing additional modules
# set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ ${CMAKE_MODULE_PATH} )

find_package(orocos_kdl)
find_package(catkin REQUIRED COMPONENTS
  orocos_kdl # use KDL functionalities
  roscpp
  kdl_parser
  std_msgs
)

link_libraries(pthread)

catkin_package(INCLUDE_DIRS
  include
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(ur10-robot
  src/ur10_robot/ur10_robot.cpp
  src/ur10_robot/ur10_model.cpp
)

add_dependencies(ur10-robot
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(ur10-robot
  ${catkin_LIBRARIES}
  armadillo
)
