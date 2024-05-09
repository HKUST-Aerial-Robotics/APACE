cmake_minimum_required(VERSION 2.8.3)
project(trajectory)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -w")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(NLopt REQUIRED)
set(NLopt_INCLUDE_DIRS ${NLOPT_INCLUDE_DIR})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
  message_generation
  active_perception
  plan_env
  voxel_mapping
  pcl_conversions
)
find_package(NLopt REQUIRED)
set(NLopt_INCLUDE_DIRS ${NLOPT_INCLUDE_DIR})

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Bspline.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES trajectory
  CATKIN_DEPENDS message_runtime active_perception plan_env voxel_mapping

  # DEPENDS system_lib
)
 
include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIR}
)

add_library(trajectory
  src/bspline/bspline_optimizer.cpp
  src/bspline/non_uniform_bspline.cpp
  src/polynomial/polynomial_traj.cpp
  src/traj_utils/planning_visualization.cpp
  src/traj_utils/process_msg.cpp
  src/traj_visibility.cpp
  src/parallax_util.cpp
)
target_link_libraries(trajectory
  ${catkin_LIBRARIES}
  ${NLOPT_LIBRARIES}
)

add_executable(process_msg
  src/traj_utils/process_msg.cpp
  src/bspline/non_uniform_bspline.cpp
)
target_link_libraries(process_msg
  ${catkin_LIBRARIES}
)