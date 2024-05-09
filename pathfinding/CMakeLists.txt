cmake_minimum_required(VERSION 2.8.3)
project(pathfinding)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
  voxel_mapping
  plan_env
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pathfinding
  CATKIN_DEPENDS voxel_mapping plan_env
  # DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

add_library(pathfinding 
  src/astar.cpp
  src/kinodynamic_astar.cpp 
  src/topo_prm.cpp
)
target_link_libraries(pathfinding
  ${catkin_LIBRARIES} 
)  


