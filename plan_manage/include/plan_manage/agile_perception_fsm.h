#ifndef _ACTIVE_LOCAL_FSM_H_
#define _ACTIVE_LOCAL_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <bspline/bspline_optimizer.h>
#include <pathfinding/kinodynamic_astar.h>
#include <plan_env/edt_environment.h>
#include <plan_env/obj_predictor.h>
#include <plan_manage/planner_manager.h>
#include <tic_toc.h>
#include <traj_utils/planning_visualization.h>
#include <trajectory/Bspline.h>

using std::vector;

namespace fast_planner {
class AgilePerceptionFSM {
private:
  /* ---------- flag ---------- */
  enum FSM_EXEC_STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, EXEC_TRAJ, REPLAN_NEW };
  enum TARGET_TYPE { MANUAL_TARGET = 1, PRESET_TARGET = 2, REFENCE_PATH = 3 };

  /* planning utils */
  FastPlannerManager::Ptr planner_manager_;
  PlanningVisualization::Ptr visualization_;

  /* parameters */
  int target_type_; // 1 mannual select, 2 hard code
  double no_replan_thresh_, replan_thresh_;
  double waypoints_[50][3];
  int waypoint_num_;

  /* planning data */
  bool trigger_, have_target_, have_odom_;
  FSM_EXEC_STATE exec_state_;

  Eigen::Vector3d odom_pos_, odom_vel_; // odometry state
  Eigen::Quaterniond odom_orient_;

  Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_; // start state
  Eigen::Vector3d end_pt_, end_vel_, end_yaw_;                   // target state
  int current_wp_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, test_something_timer_;
  ros::Subscriber waypoint_sub_, odom_sub_;
  ros::ServiceServer map_save_service_, map_load_service_, start_eval_service_, end_eval_service_;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_;

  /* Error evaluate */
  bool start_eval_;
  bool end_eval_;
  double beginning_time_;
  double beginning_error_;
  double last_eval_time_;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry>
      SyncPolicyErrorEvaluate;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyErrorEvaluate>>
      SynchronizerErrorEvaluate;
  shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> airsim_sync_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> vins_sync_sub_;
  SynchronizerErrorEvaluate sync_error_evaluate_;

  /* ROS callbacks */
  void execFSMCallback(const ros::TimerEvent &e);
  void waypointCallback(const nav_msgs::PathConstPtr &msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
  void evaluateCallback(const nav_msgs::OdometryConstPtr &airsim_msg,
                        const nav_msgs::OdometryConstPtr &vins_msg);

  /* ROS services */
  bool saveMapCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);
  bool loadMapCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);
  bool startEvalCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);
  bool endEvalCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);

  /* Helper functions */
  void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
  void printFSMExecState();
  bool callAgilePerceptionReplan(); // front-end and back-end method

public:
  AgilePerceptionFSM(/* args */) {}
  ~AgilePerceptionFSM() {}

  void init(ros::NodeHandle &nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace fast_planner

#endif