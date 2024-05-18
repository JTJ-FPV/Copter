cmake_minimum_required(VERSION 3.0.2)
project(grid_search)

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

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  sensor_msgs
  roscpp
  std_msgs
  geometry_msgs
  visualization_msgs
  message_filters
  occupancy_grid_map
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES grid_search
 CATKIN_DEPENDS nav_msgs roscpp std_msgs visualization_msgs occupancy_grid_map
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

add_library( grid_search
  src/Astar_search.cpp
)

target_link_libraries(grid_search
  ${catkin_LIBRARIES}
)

add_executable(Astar_search_test src/Astar_search_test.cpp)

target_link_libraries(Astar_search_test
  ${catkin_LIBRARIES}
  grid_search
)

add_executable(rdp src/rdp.cpp)
target_link_libraries(rdp
  ${catkin_LIBRARIES}
)