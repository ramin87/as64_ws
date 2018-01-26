cmake_minimum_required(VERSION 2.8.3)
project(io_test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  io_lib
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
	""
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
)

# *************************************************
# *************************************************

add_executable(file_io_test
  src/file_io_test.cpp ${INCLUDE_LIST} ${SRC_LIST}
)

add_dependencies(file_io_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(file_io_test
  ${catkin_LIBRARIES}
  io-lib
)


# *************************************************
# *************************************************
