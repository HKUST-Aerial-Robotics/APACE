
#include <plan_manage/agile_perception_fsm.h>

namespace fast_planner {
void AgilePerceptionFSM::init(ros::NodeHandle &nh) {
  current_wp_ = 0;
  exec_state_ = FSM_EXEC_STATE::INIT;
  have_target_ = false;
  have_odom_ = false;
  start_eval_ = false;
  end_eval_ = false;
  beginning_time_ = 0.0;
  beginning_error_ = 0.0;
  last_eval_time_ = -1.0;

  /* Fsm param */
  nh.param("fsm/flight_type", target_type_, -1);
  nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
  nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
  nh.param("fsm/wp_num", waypoint_num_, -1);
  for (int i = 0; i < waypoint_num_; i++) {
    nh.param("fsm/wp" + to_string(i) + "_x", waypoints_[i][0], -1.0);
    nh.param("fsm/wp" + to_string(i) + "_y", waypoints_[i][1], -1.0);
    nh.param("fsm/wp" + to_string(i) + "_z", waypoints_[i][2], -1.0);
  }

  /* Initialize main modules */
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);
  visualization_.reset(new PlanningVisualization(nh));

  /* ROS utils */
  exec_timer_ = nh.createTimer(ros::Duration(0.01), &AgilePerceptionFSM::execFSMCallback, this);
  // safety_timer_ =
  //     nh.createTimer(ros::Duration(0.05), &AgilePerceptionFSM::checkCollisionCallback, this);

  waypoint_sub_ =
      nh.subscribe("/waypoint_generator/waypoints", 1, &AgilePerceptionFSM::waypointCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &AgilePerceptionFSM::odometryCallback, this);

  replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  new_pub_ = nh.advertise<std_msgs::Empty>("/planning/new", 10);
  bspline_pub_ = nh.advertise<trajectory::Bspline>("/planning/bspline", 10);

  map_save_service_ =
      nh.advertiseService("/services/save_map", &AgilePerceptionFSM::saveMapCallback, this);
  map_load_service_ =
      nh.advertiseService("/services/load_map", &AgilePerceptionFSM::loadMapCallback, this);
  // start_eval_service_ =
  //     nh.advertiseService("/services/start_eval", &AgilePerceptionFSM::startEvalCallback, this);
  // end_eval_service_ =
  //     nh.advertiseService("/services/end_eval", &AgilePerceptionFSM::endEvalCallback, this);

  airsim_sync_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(
      nh, "/airsim_node/drone_1/odom_local_enu", 25));
  vins_sync_sub_.reset(
      new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/vins_estimator/odometry", 25));
  sync_error_evaluate_.reset(new message_filters::Synchronizer<SyncPolicyErrorEvaluate>(
      SyncPolicyErrorEvaluate(100), *airsim_sync_sub_, *vins_sync_sub_));
  // sync_error_evaluate_->registerCallback(
  //     boost::bind(&AgilePerceptionFSM::evaluateCallback, this, _1, _2));

  // Preload occupancy and esdf map
  planner_manager_->loadMapService();
}

/* ------------------------------ ROS Callbacks ----------------------------- */

void AgilePerceptionFSM::execFSMCallback(const ros::TimerEvent &e) {
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100) {
    // printFSMExecState();
    if (!have_odom_)
      cout << "no odom." << endl;
    // if (!trigger_)
    //   cout << "wait for goal." << endl;
    fsm_num = 0;
  }

  switch (exec_state_) {
  case INIT: {
    if (!have_odom_) {
      return;
    }
    if (!trigger_) {
      return;
    }
    changeFSMExecState(WAIT_TARGET, "FSM");
    break;
  }

  case WAIT_TARGET: {
    if (!have_target_)
      return;
    else {
      changeFSMExecState(GEN_NEW_TRAJ, "FSM");
    }
    break;
  }

  case GEN_NEW_TRAJ: {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
    start_yaw_(0) = atan2(rot_x(1), rot_x(0));
    start_yaw_(1) = start_yaw_(2) = 0.0;

    bool success = callAgilePerceptionReplan();
    if (success) {
      start_eval_ = true;
      changeFSMExecState(EXEC_TRAJ, "FSM");
    } else {
      // have_target_ = false;
      // changeFSMExecState(WAIT_TARGET, "FSM");
      changeFSMExecState(GEN_NEW_TRAJ, "FSM");
    }
    break;
  }

  case EXEC_TRAJ: {
    /* determine if need to replan */
    LocalTrajData *info = &planner_manager_->local_data_;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();
    t_cur = min(info->duration_, t_cur);

    Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

    /* && (end_pt_ - pos).norm() < 0.5 */
    if (t_cur > info->duration_ - 1e-2) {
      have_target_ = false;
      end_eval_ = true;
      changeFSMExecState(WAIT_TARGET, "FSM");
      return;
    } 
    break;
  }
  }
}

void AgilePerceptionFSM::waypointCallback(const nav_msgs::PathConstPtr &msg) {
  trigger_ = true;

  if (target_type_ == TARGET_TYPE::MANUAL_TARGET) {
    if (msg->poses[0].pose.position.z < -0.1)
      return;

    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;
    Eigen::Quaterniond end_pt_quad;

    end_pt_quad.w() = msg->poses[0].pose.orientation.w;
    end_pt_quad.x() = msg->poses[0].pose.orientation.x;
    end_pt_quad.y() = msg->poses[0].pose.orientation.y;
    end_pt_quad.z() = msg->poses[0].pose.orientation.z;

    Eigen::Vector3d rot_x = end_pt_quad.toRotationMatrix().block(0, 0, 3, 1);
    end_yaw_(0) = atan2(rot_x(1), rot_x(0));
    end_yaw_(1) = end_yaw_(2) = 0.0;

  } else if (target_type_ == TARGET_TYPE::PRESET_TARGET) {
    end_pt_(0) = waypoints_[current_wp_][0];
    end_pt_(1) = waypoints_[current_wp_][1];
    end_pt_(2) = waypoints_[current_wp_][2];
    current_wp_ = (current_wp_ + 1) % waypoint_num_;
  }

  visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
  end_vel_.setZero();
  have_target_ = true;

  if (exec_state_ == WAIT_TARGET)
    changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
}

void AgilePerceptionFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg) {
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  have_odom_ = true;
}

void AgilePerceptionFSM::evaluateCallback(const nav_msgs::OdometryConstPtr &airsim_msg,
                                          const nav_msgs::OdometryConstPtr &vins_msg) {
  if (!start_eval_)
    return;

  if (beginning_time_ < 1e-4)
    beginning_time_ = ros::Time::now().toSec();

  const double z_offset = 0.57;

  double time_now = ros::Time::now().toSec() - beginning_time_;
  double time_diff = time_now - last_eval_time_;

  if (time_diff > 0.2) {
    last_eval_time_ = time_now;
    Eigen::Vector3d airsim_pos(airsim_msg->pose.pose.position.x, airsim_msg->pose.pose.position.y,
                               airsim_msg->pose.pose.position.z + z_offset);
    Eigen::Vector3d vins_pos(vins_msg->pose.pose.position.x, vins_msg->pose.pose.position.y,
                             vins_msg->pose.pose.position.z);

    double est_error = (airsim_pos - vins_pos).norm();

    if (beginning_error_ < 1e-4)
      beginning_error_ = est_error;

    est_error -= beginning_error_;

    // Write eval result into txt file
    // if (time_now < 0.1) {
    //   std::ofstream ofs("/home/cindy/workspace/Agile_Perception_Aware_ws/eval.txt");
    //   ofs.close();
    // }
    // std::ofstream ofs("/home/cindy/workspace/Agile_Perception_Aware_ws/eval.txt", std::ios::app);
    // if (!ofs.is_open()) {
    //   std::cerr << "Failed to open file: eval.txt" << std::endl;
    //   return;
    // }
    // ofs << time_now << "," << est_error << std::endl;
    // ofs.close();

    // Termination condition
    Eigen::Vector3d airsim_vel(airsim_msg->twist.twist.linear.x, airsim_msg->twist.twist.linear.y,
                               airsim_msg->twist.twist.linear.z);
    double vel = airsim_vel.norm();
    double dist_goal = (airsim_pos - end_pt_).norm();

    if ((airsim_vel.norm() < 0.1 && dist_goal < 1.0) || end_eval_) {
      ROS_ERROR("[evaluateCallback] Estimation error: %f", est_error);
      start_eval_ = false;
    } else
      ROS_WARN("[evaluateCallback] Estimation error: %f", est_error);
  }
}
/* ------------------------------ ROS Services ------------------------------ */

bool AgilePerceptionFSM::saveMapCallback(std_srvs::Empty::Request &request,
                                         std_srvs::Empty::Response &response) {
  planner_manager_->saveMapService();
  return true;
}

bool AgilePerceptionFSM::loadMapCallback(std_srvs::Empty::Request &request,
                                         std_srvs::Empty::Response &response) {
  planner_manager_->loadMapService();
  return true;
}

bool AgilePerceptionFSM::startEvalCallback(std_srvs::Empty::Request &request,
                                           std_srvs::Empty::Response &response) {
  ROS_WARN("received service topic");
  start_eval_ = true;
  return true;
}

bool AgilePerceptionFSM::endEvalCallback(std_srvs::Empty::Request &request,
                                         std_srvs::Empty::Response &response) {
  end_eval_ = true;
  return true;
}

/* ---------------------------- Helper Functions ---------------------------- */

void AgilePerceptionFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call) {
  string state_str[5] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "EXEC_TRAJ"};
  int pre_s = int(exec_state_);
  exec_state_ = new_state;
  // cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)]
  //      << endl;
}

void AgilePerceptionFSM::printFSMExecState() {
  string state_str[5] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "EXEC_TRAJ"};

  cout << "[FSM] state: " + state_str[int(exec_state_)] << endl;
}

bool AgilePerceptionFSM::callAgilePerceptionReplan() {
  ros::Time time_r = ros::Time::now();
  bool truncated = false;
  TicToc t_replan;

  // Compute time lower bound of yaw and use in trajectory generation
  double diff = fabs(end_yaw_(0) - start_yaw_(0));
  double max_yaw_vel = 60 * 3.1415926 / 180.0;
  double time_lb = min(diff, 2 * M_PI - diff) / max_yaw_vel;

  int plan_success = planner_manager_->planLocalMotion(end_pt_, start_pt_, start_vel_, start_acc_,
                                                       truncated, time_lb);
  double replan_time = t_replan.toc();
  ROS_WARN("[Local Planner] Plan local motion time: %fs", replan_time);

  if (plan_success == LOCAL_FAIL) {
    ROS_ERROR("planLocalMotion fail.");
    return false;
  }

  TicToc t_yaw;

  bool specify_end_yaw = (truncated) ? false : true;

  // planner_manager_->planYawPreset(start_yaw_, 0.0);
  // planner_manager_->planYawPreset(start_yaw_, -3.14);
  planner_manager_->planYawCovisibility();
  // planner_manager_->planYawPercepAgnostic();

  ROS_WARN("[Local Planner] Plan yaw time: %fs", t_yaw.toc());

  auto info = &planner_manager_->local_data_;
  // info->start_time_ = time_r;
  info->start_time_ = ros::Time::now();

  /* publish traj */
  trajectory::Bspline bspline;
  bspline.order = planner_manager_->pp_.bspline_degree_;
  bspline.start_time = info->start_time_;
  bspline.traj_id = info->traj_id_;

  Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();

  for (int i = 0; i < pos_pts.rows(); ++i) {
    geometry_msgs::Point pt;
    pt.x = pos_pts(i, 0);
    pt.y = pos_pts(i, 1);
    pt.z = pos_pts(i, 2);
    bspline.pos_pts.push_back(pt);
  }

  Eigen::VectorXd knots = info->position_traj_.getKnot();
  for (int i = 0; i < knots.rows(); ++i) {
    bspline.knots.push_back(knots(i));
  }

  Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
  for (int i = 0; i < yaw_pts.rows(); ++i) {
    double yaw = yaw_pts(i, 0);
    bspline.yaw_pts.push_back(yaw);
  }
  bspline.yaw_dt = info->yaw_traj_.getKnotSpan();

  bspline_pub_.publish(bspline);

  /* visulization */
  auto plan_data = &planner_manager_->plan_data_;

  visualization_->drawGeometricPath(plan_data->kino_path_, 0.05, Eigen::Vector4d(1, 0, 1, 1));
  visualization_->drawBspline(info->position_traj_, 0.08, Eigen::Vector4d(1.0, 1.0, 0.0, 1), true,
                              0.15, Eigen::Vector4d(1, 0, 0, 0.5), false, 0.15,
                              Eigen::Vector4d(0, 1, 0, 0.5));
  visualization_->drawYawOnKnots(info->position_traj_, info->acceleration_traj_, info->yaw_traj_);

  return true;
}

} // namespace fast_planner
