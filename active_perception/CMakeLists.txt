cmake_minimum_required(VERSION 2.8.3)
project(active_perception)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -w")


find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
  plan_env
  pathfinding
  voxel_mapping
  exploration_utils
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES active_perception
 CATKIN_DEPENDS plan_env pathfinding voxel_mapping exploration_utils
 DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

add_library(active_perception
  src/heading_planner.cpp
  src/frontier_finder.cpp
  src/graph_node.cpp
  src/perception_utils.cpp
  src/yaw_graph_utils.cpp
)
target_link_libraries(active_perception
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
)  
