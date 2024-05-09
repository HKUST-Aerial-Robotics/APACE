#ifndef _TRAJ_SERVER_H_
#define _TRAJ_SERVER_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <active_perception/perception_utils.h>
#include <bspline/non_uniform_bspline.h>
#include <polynomial/polynomial_traj.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <trajectory/Bspline.h>

using fast_planner::NonUniformBspline;
using fast_planner::PerceptionUtils;
using fast_planner::Polynomial;
using fast_planner::PolynomialTraj;

namespace fast_planner {
class TrajServer {
private:
  /* ------------------------------- Attributes ------------------------------- */
  ros::Publisher cmd_vis_pub, pos_cmd_pub, traj_pub, trigger_pub;
  ros::Subscriber bspline_sub, replan_sub, new_sub, odom_sub;
  ros::ServiceServer start_eval_service_;
  ros::ServiceClient start_cnt_client_, end_cnt_client_;
  ros::Timer cmd_timer, vis_timer;
  nav_msgs::Odometry odom;
  quadrotor_msgs::PositionCommand cmd;

  // Info of generated traj
  vector<NonUniformBspline> traj_;
  double traj_duration_;
  ros::Time start_time_;
  int traj_id_;
  int pub_traj_id_;

  shared_ptr<PerceptionUtils> percep_utils_;

  // Info of replan
  bool receive_traj_ = false;
  bool traj_ready_ = false;
  double replan_time_;

  // Executed traj, commanded and real ones
  vector<Eigen::Vector3d> traj_cmd_, traj_real_;
  vector<double> yaw_traj_cmd_, yaw_traj_real_;

  // Initialization
  Eigen::Vector3d init_pos;
  double init_yaw;
  double init_sleep_time;

  // Evaluation data
  ros::Time start_time, end_time, last_time;
  double energy;
  double max_vel;
  double max_acc;
  double cur_vel;
  double cur_acc;

  // Error evaluate
  bool start_eval_;
  double start_eval_time_;
  double last_eval_time_;
  double last_eval_error_;
  double RMSE_;
  int num_eval_;
  Eigen::Vector3d error_bias_;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry>
      SyncPolicyErrorEvaluate;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyErrorEvaluate>>
      SynchronizerErrorEvaluate;
  shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> airsim_sync_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> vins_sync_sub_;
  SynchronizerErrorEvaluate sync_error_evaluate_;

  /* -------------------------------- Functions ------------------------------- */
  // ROS callbacks
  void replanCallback(std_msgs::Empty msg);
  void newCallback(std_msgs::Empty msg);
  void odomCallbck(const nav_msgs::Odometry &msg);
  void visCallback(const ros::TimerEvent &e);
  void bsplineCallback(const trajectory::BsplineConstPtr &msg);
  void cmdCallback(const ros::TimerEvent &e);
  void evaluateCallback(const nav_msgs::OdometryConstPtr &airsim_msg,
                        const nav_msgs::OdometryConstPtr &vins_msg);
  bool startEvalCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);

  // Helper functions
  void startTrigger();
  double calcPathLength(const vector<Eigen::Vector3d> &path);
  void prepareYaw(Eigen::Vector3d pos, double yaw_cmd, double yaw_cur);

  // Visualization
  void displayTrajWithColor(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color,
                            int id);
  void drawFOV(const vector<Eigen::Vector3d> &list1, const vector<Eigen::Vector3d> &list2);
  void drawCmd(const Eigen::Vector3d &pos, const Eigen::Vector3d &vec, const int &id,
               const Eigen::Vector4d &color);

  // Test functions
  void fitBsplineAndExecute(Eigen::MatrixXd &waypoints, Eigen::VectorXd &times);
  void sin_curve_traj();
  void straight_line_traj();

public:
  TrajServer() {}
  ~TrajServer() {}

  void init(ros::NodeHandle &nh);
};
} // namespace fast_planner
#endif