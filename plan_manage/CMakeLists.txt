cmake_minimum_required(VERSION 2.8.3)
project(plan_manage)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -w")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  cv_bridge
  plan_env
  pathfinding
  trajectory
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES plan_manage
 CATKIN_DEPENDS plan_env pathfinding trajectory)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


add_library(plan_manage 
src/planner_manager.cpp
src/yaw_initial_planner.cpp
)
target_link_libraries(plan_manage ${catkin_LIBRARIES})

add_executable(fast_planner_node
  src/fast_planner_node.cpp 
  src/agile_perception_fsm.cpp
  src/planner_manager.cpp
  src/yaw_initial_planner.cpp
  src/yaw_graph_utils.cpp
)
target_link_libraries(fast_planner_node 
  ${catkin_LIBRARIES}
  -ldw
  -lglog
)

add_executable(traj_server 
  src/traj_server_node.cpp
  src/traj_server.cpp
)
target_link_libraries(traj_server 
  ${catkin_LIBRARIES} 
  -ldw 
  -lglog
)