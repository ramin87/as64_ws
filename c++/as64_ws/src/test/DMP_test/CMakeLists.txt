cmake_minimum_required(VERSION 2.8.3)
project(dmp_test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  #OpenCV REQUIRED
  #PCL REQUIRED
  roscpp
  roslib
  DMP_lib
  io_lib
  param_lib
  signalProcessing_lib
  plot_lib
  math_lib
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

add_executable(DMP_canonicalClock_test
  src/DMP_canonicalClock_test.cpp ${INCLUDE_LIST} ${SRC_LIST}
)

add_dependencies(DMP_canonicalClock_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(DMP_canonicalClock_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  DMP-lib
  io-lib
)


# *************************************************
# *************************************************

add_executable(DMP_gatingFunction_test
  src/DMP_gatingFunction_test.cpp ${INCLUDE_LIST} ${SRC_LIST}
)

add_dependencies(DMP_gatingFunction_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(DMP_gatingFunction_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  DMP-lib
  io-lib
)

# *************************************************
# *************************************************

add_executable(DMP_test
  src/DMP_test.cpp ${INCLUDE_LIST} ${SRC_LIST}
)

add_dependencies(DMP_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(DMP_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  DMP-lib
  io-lib
  param-lib
  signalProcessing-lib
  plot-lib
)

# *************************************************
# *************************************************

add_executable(DMP_CartPos_test
  src/DMP_CartPos_test.cpp ${INCLUDE_LIST} ${SRC_LIST}
)

add_dependencies(DMP_CartPos_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(DMP_CartPos_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  DMP-lib
  io-lib
  param-lib
  signalProcessing-lib
  plot-lib
)

# *************************************************
# *************************************************

add_executable(DMP_orient_test
  src/DMP_orient_test.cpp ${INCLUDE_LIST} ${SRC_LIST}
)

add_dependencies(DMP_orient_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(DMP_orient_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  DMP-lib
  io-lib
  param-lib
  signalProcessing-lib
  plot-lib
  math-lib
)
