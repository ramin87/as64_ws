cmake_minimum_required(VERSION 2.8.3)
project(dmp_ur10_controller)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  #OpenCV REQUIRED
  #PCL REQUIRED
  roscpp
  roslib
  dmp_lib
  io_lib
  param_lib
  sigproc_lib
  plot_lib
  math_lib
  time_lib
  ur10_robot
)

# BHand need to know that we are on Linux
add_definitions( -DLINUX )


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  DEPENDS opencv pcl eigen
)

set(SRC_LIST
	src/source_list/utils.cpp
	src/source_list/cmd_args.cpp
	src/source_list/log_data.cpp
  src/source_list/DMP_UR10_controller.cpp
)

file(GLOB INCLUDE_LIST "")

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3
)

# *************************************************
# *************************************************

add_executable(DMP_UR10_controller
  src/main.cpp ${INCLUDE_LIST} ${SRC_LIST}
)

add_dependencies(DMP_UR10_controller
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(DMP_UR10_controller
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  dmp-lib
  io-lib
  param-lib
  sigproc-lib
  plot-lib
  math-lib
  ur10-robot
  time-lib
)
