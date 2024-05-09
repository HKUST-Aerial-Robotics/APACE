#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/agile_perception_fsm.h>

#include <backward/backward.hpp>
namespace backward {
backward::SignalHandling sh;
}

using namespace fast_planner;

int main(int argc, char **argv) {
  ros::init(argc, argv, "fast_planner_node");
  ros::NodeHandle nh("~");

  int planner;
  nh.param("planner_node/planner", planner, -1);

  AgilePerceptionFSM agile_perception;

  // if (planner == 1) {
  //   kino_replan.init(nh);
  // } else if (planner == 2) {
  //   topo_replan.init(nh);
  // } else if (planner == 3) {
  //   active_local.init(nh);
  // }

  agile_perception.init(nh);


  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}
