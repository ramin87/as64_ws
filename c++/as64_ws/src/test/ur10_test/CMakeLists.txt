cmake_minimum_required(VERSION 2.8.3)
project(ur10_test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  math_lib
  param_lib
  io_lib
  ur10_robot
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  DEPENDS
)

set(SRC_LIST
	src/source_list/cmd_args.cpp
)

file(GLOB INCLUDE_LIST "")

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
)

SET(BUILD_LIBS
  ${catkin_LIBRARIES}
  armadillo
  math-lib
  param-lib
  io-lib
  ur10-robot
)

# *************************************************
# *************************************************

add_executable(ur10_test src/ur10_test.cpp ${INCLUDE_LIST} ${SRC_LIST})
add_dependencies(ur10_test ${${PROJECT_NAME}_EXPORTED_TARGETS}M${catkin_EXPORTED_TARGETS})
target_link_libraries(ur10_test ${BUILD_LIBS})

# *************************************************
# *************************************************

add_executable(run_urscript src/run_urscript.cpp ${INCLUDE_LIST} ${SRC_LIST} )
add_dependencies(run_urscript ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(run_urscript ${BUILD_LIBS} )


# *************************************************
# *************************************************

add_executable(test_force_mode src/test_force_mode.cpp ${INCLUDE_LIST} ${SRC_LIST} )
add_dependencies(test_force_mode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_force_mode ${BUILD_LIBS})

# *************************************************
# *************************************************

add_executable(test_logging src/test_logging.cpp ${INCLUDE_LIST} ${SRC_LIST} )
add_dependencies(test_logging ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_logging ${BUILD_LIBS})

# *************************************************
# *************************************************

add_executable(test_force_mode_with_data_logging src/test_force_mode_with_data_logging.cpp ${INCLUDE_LIST} ${SRC_LIST} )
add_dependencies(test_force_mode_with_data_logging ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_force_mode_with_data_logging ${BUILD_LIBS})

# *************************************************
# *************************************************
