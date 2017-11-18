cmake_minimum_required(VERSION 2.8.3)
project(handover)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


#SET(DLIB_VERSION "18.18")
#if (NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/dlib/cmake")
#file(MAKE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/dlib-extracted")
#file(DOWNLOAD "http://dlib.net/files/dlib-${DLIB_VERSION}.tar.bz2" "${CMAKE_CURRENT_SOURCE_DIR}/dlib.tar.bz2")
#execute_process(
#COMMAND ${CMAKE_COMMAND} -E tar xjf ${CMAKE_CURRENT_SOURCE_DIR}/dlib.tar.bz2
#WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/dlib-extracted"
#)
#file(REMOVE "${CMAKE_CURRENT_SOURCE_DIR}/dlib.tar.bz2")
#file(RENAME "${CMAKE_CURRENT_SOURCE_DIR}/dlib-extracted/dlib-${DLIB_VERSION}/dlib" "${CMAKE_CURRENT_SOURCE_DIR}/dlib")
#file(REMOVE_RECURSE "${CMAKE_CURRENT_SOURCE_DIR}/dlib-extracted")
#endif()
#include(dlib/cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  autharl_core
  lwr_robot
  DS_lib
  bhand_lib
  math_lib
  filter_lib
  time_lib
  robotics_lib
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

# BHand need to know that we are on Linux
add_definitions( -DLINUX )

## Generate messages in the 'msg' folder
add_message_files(
   DIRECTORY msg FILES Num.msg
#   Message1.msg
#   Message2.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   AddTwoInts.srv
#   Service1.srv
#   Service2.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES SEDS
  CATKIN_DEPENDS roscpp std_msgs message_runtime
  DEPENDS opencv pcl eigen
#  DEPENDS system_lib
)

set(SRC_LIST
	src/source_list/utils.cpp
)

file(GLOB INCLUDE_LIST "include/SEDS/*.h" "include/SEDS/*.hpp")

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  #${ARMADILLO_INCLUDE_DIR}
  #/home/user/workspace/cpp/lib/fril/include
  #/usr/local/include/armadillo_bits
  #/home/user/workspace/armadillo-4.400.2/include/armadillo_bits
  ${Boost_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# *************************************************
# *************************************************

add_executable(handover_node
  src/main.cpp ${INCLUDE_LIST} ${SRC_LIST}
  src/arm_handover_controller.cpp
  src/handover_controller.cpp)

add_dependencies(handover_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(handover_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  autharl-robot
  autharl-utils
  lwr-robot
  DS-lib
  math-lib
  filter-lib
  time-lib
  robotics-lib
)

# *************************************************
# *************************************************

add_executable(bhand_node
  src/bad.cpp
  ${INCLUDE_LIST}
  ${SRC_LIST}
)

add_dependencies(bhand_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  bad_generate_messages_cpp
)

target_link_libraries(bhand_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  bhand
)

add_executable(bhand_stop
  src/bad_stop.cpp
  ${INCLUDE_LIST}
  ${SRC_LIST}
)

add_dependencies(bhand_stop
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  bad_generate_messages_cpp
)

target_link_libraries(bhand_stop
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  bhand
)

# *************************************************
# *************************************************

add_executable(test_DS_motion
  src/test_DS_motion.cpp ${INCLUDE_LIST} ${SRC_LIST}
)

add_dependencies(test_DS_motion
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(test_DS_motion
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  autharl-robot
  autharl-utils
  lwr-robot
  DS-lib
  math-lib
  filter-lib
  time-lib
  robotics-lib
)

# *************************************************
# *************************************************
