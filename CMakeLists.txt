cmake_minimum_required(VERSION 2.8.3)
project(polygon_gridmap)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED roscpp
  grid_map_ros
  grid_map_msgs
)

find_package(yaml-cpp REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES polygon_grid_map
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${catkin_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR})

add_executable(
  polygon_to_grid_map
  src/polygon_to_grid_map.cpp
)

target_link_libraries(
  polygon_to_grid_map
  ${YAML_CPP_LIBRARIES}
  ${catkin_LIBRARIES}
)