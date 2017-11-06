cmake_minimum_required(VERSION 2.8.3)
project(dmp_test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  as64_utils
  dmp_lib
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

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
)

# *************************************************
# *************************************************

add_executable(DMP_test_node
  src/main.cpp ${INCLUDE_LIST} ${SRC_LIST}
)

add_dependencies(DMP_test_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(DMP_test_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  DMP-lib
  as64-filter
)


# *************************************************
# *************************************************
