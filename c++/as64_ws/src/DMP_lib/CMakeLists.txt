cmake_minimum_required(VERSION 2.8.3)

project(dmp_lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(DMP-lib
  src/DMP/DMPBase.cpp
  src/DMP/DMP.cpp
  src/DMP/DMPBio.cpp
  src/CanonicalSystem/CanonicalSystem.cpp
  src/CanonicalSystem/LinCanonicalSystem.cpp
  src/CanonicalSystem/ExpCanonicalSystem.cpp
  src/CanonicalSystem/SpringDamperCanonicalSystem.cpp
)

add_dependencies(DMP-lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(DMP-lib
  ${catkin_LIBRARIES}
  armadillo
)
