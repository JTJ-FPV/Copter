cmake_minimum_required(VERSION 3.0.2)
project(occupancy_grid_map)

# SET(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  message_filters
  roscpp
  rospy
  std_msgs
  visualization_msgs
)

find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES occupancy_grid_map
 CATKIN_DEPENDS cv_bridge message_filters roscpp rospy std_msgs visualization_msgs
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
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library(occupancy_grid_map
  src/occ_grid_map.cpp
  src/raycast.cpp
)

target_link_libraries(occupancy_grid_map
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)


add_executable(occ_grid_map_node src/occ_grid_map_node.cpp)


target_link_libraries(occ_grid_map_node
  ${catkin_LIBRARIES}
  occupancy_grid_map
)

# add_dependencies(occupany_grid_map_node ${catkin_EXPORTED_TARGETS})
