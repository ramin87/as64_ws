cmake_minimum_required(VERSION 2.8.3)
project(robot_grav_comp_move)

#find_package(Armadillo  7.500 REQUIRED )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#set(VISION_DIR "")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  autharl_core
  std_msgs
  lwr_robot
  io_lib
  time_lib
)

catkin_package()

include_directories(INCLUDE_DIRS
  include
  ${catkin_INCLUDE_DIRS})

add_executable(robot_grav_comp_move_node
  src/main.cpp
  src/robot_grav_comp_controller.cpp
  src/source_list/utils.cpp)

add_dependencies(robot_grav_comp_move_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(robot_grav_comp_move_node
  ${catkin_LIBRARIES}
  autharl-robot
  autharl-viz
  lwr-robot
  pthread
  IO-lib
  time-lib
)
