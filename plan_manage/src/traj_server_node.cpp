#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/traj_server.h>

#include <backward/backward.hpp>
namespace backward {
backward::SignalHandling sh;
}

using namespace fast_planner;

int main(int argc, char **argv) {
  ros::init(argc, argv, "traj_server_node");
  ros::NodeHandle nh("~");

  TrajServer traj_server;
  traj_server.init(nh);

  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}
