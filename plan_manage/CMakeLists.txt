cmake_minimum_required(VERSION 3.0.2)
project(plan_manage)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# SET(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  grid_search
  mavros_msgs
  occupancy_grid_map
  roscpp
  sensor_msgs
  nav_msgs
  std_msgs
  message_filters
  tf2
  trajectory_gen
  trajectory_msgs
  visualization_msgs
)

find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES plan_manage
 CATKIN_DEPENDS geometry_msgs grid_search mavros_msgs occupancy_grid_map roscpp sensor_msgs std_msgs tf2 trajectory_gen trajectory_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
  "/usr/include/eigen3"
)

## Declare a C++ library
add_library(plan_manage
  src/plan_fsm.cpp
)

target_link_libraries(plan_manage
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

add_executable(plan_fsm_node src/plan_fsm_node.cpp)

target_link_libraries(plan_fsm_node
  ${catkin_LIBRARIES}
  plan_manage
)
