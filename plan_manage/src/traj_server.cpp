#include <plan_manage/traj_server.h>

namespace fast_planner {

void TrajServer::init(ros::NodeHandle &nh) {
  traj_id_ = 0;

  energy, max_vel, max_acc, cur_vel, cur_acc = 0.0;

  start_eval_ = false;
  last_eval_time_ = -1.0;
  last_eval_error_ = 0.0;
  RMSE_ = 0.0;
  num_eval_ = 0;
  error_bias_ = Eigen::Vector3d::Zero();

  // Traj server params
  nh.param("traj_server/pub_traj_id", pub_traj_id_, -1);
  nh.param("fsm/replan_time", replan_time_, 0.1);

  // Initial pose
  nh.param("traj_server/init_sleep_time", init_sleep_time, 0.0);
  nh.param("traj_server/init_x", init_pos[0], 0.0);
  nh.param("traj_server/init_y", init_pos[1], 0.0);
  nh.param("traj_server/init_z", init_pos[2], 0.0);
  nh.param("traj_server/init_yaw", init_yaw, 0.0);

  // Control parameter
  nh.param("traj_server/kx_x", cmd.kx[0], 5.7);
  nh.param("traj_server/kx_y", cmd.kx[1], 5.7);
  nh.param("traj_server/kx_z", cmd.kx[2], 6.2);
  nh.param("traj_server/kv_x", cmd.kv[0], 3.4);
  nh.param("traj_server/kv_y", cmd.kv[1], 3.4);
  nh.param("traj_server/kv_z", cmd.kv[2], 4.0);

  // ROS utils
  bspline_sub = nh.subscribe("/planning/bspline", 10, &TrajServer::bsplineCallback, this);
  replan_sub = nh.subscribe("/planning/replan", 10, &TrajServer::replanCallback, this);
  new_sub = nh.subscribe("/planning/new", 10, &TrajServer::newCallback, this);
  odom_sub = nh.subscribe("/odom_world", 50, &TrajServer::odomCallbck, this);

  cmd_vis_pub = nh.advertise<visualization_msgs::Marker>("/planning/position_cmd_vis", 10);
  pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
  traj_pub = nh.advertise<visualization_msgs::Marker>("/planning/travel_traj", 10);
  trigger_pub = nh.advertise<nav_msgs::Path>("/waypoint_generator/waypoints", 10);

  cmd_timer = nh.createTimer(ros::Duration(0.01), &TrajServer::cmdCallback, this);
  vis_timer = nh.createTimer(ros::Duration(0.25), &TrajServer::visCallback, this);

  start_eval_service_ =
      nh.advertiseService("/services/start_eval", &TrajServer::startEvalCallback, this);
  start_cnt_client_ = nh.serviceClient<std_srvs::Empty>("/services/start_cnt");
  end_cnt_client_ = nh.serviceClient<std_srvs::Empty>("/services/end_cnt");

  airsim_sync_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(
      nh, "/airsim_node/drone_1/odom_local_enu", 25));
  vins_sync_sub_.reset(
      new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/vins_estimator/odometry", 25));
  sync_error_evaluate_.reset(new message_filters::Synchronizer<SyncPolicyErrorEvaluate>(
      SyncPolicyErrorEvaluate(100), *airsim_sync_sub_, *vins_sync_sub_));
  sync_error_evaluate_->registerCallback(boost::bind(&TrajServer::evaluateCallback, this, _1, _2));

  // Start initialization
  ROS_INFO("[Traj server] Initializing");
  ros::Duration(1.0).sleep();

  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;
  cmd.position.x = 0.0;
  cmd.position.y = 0.0;
  cmd.position.z = 0.0;
  cmd.velocity.x = 0.0;
  cmd.velocity.y = 0.0;
  cmd.velocity.z = 0.0;
  cmd.acceleration.x = 0.0;
  cmd.acceleration.y = 0.0;
  cmd.acceleration.z = 0.0;
  cmd.yaw = 0.0;
  cmd.yaw_dot = 0.0;

  percep_utils_.reset(new PerceptionUtils(nh));
  max_vel = 0.0;
  max_acc = 0.0;

  // Wait for frontiers to be ready
  ros::Duration(init_sleep_time).sleep();

  // Move to init position
  for (int i = 0; i < 100; ++i) {
    cmd.position.z += init_pos[2] / 100.0;
    cmd.yaw = init_yaw;
    pos_cmd_pub.publish(cmd);
    ros::Duration(0.01).sleep();
  }

  double dist = sqrt(init_pos[0] * init_pos[0] + init_pos[1] * init_pos[1]);
  int n = dist / 0.01;
  Eigen::Vector2d init_pos_xy(init_pos[0], init_pos[1]);
  Eigen::Vector2d init_unit = init_pos_xy.normalized() * 0.01;
  for (int i = 0; i < n; ++i) {
    cmd.position.x += init_unit[0];
    cmd.position.y += init_unit[1];
    cmd.yaw = init_yaw;
    pos_cmd_pub.publish(cmd);
    ros::Duration(0.01).sleep();
  }
  ros::Duration(1.0).sleep();

  ros::Duration(2.5).sleep();
  ROS_INFO("[Traj server] Initilization finished");

  startTrigger();
  // straight_line_traj();
}

/* ------------------------------ ROS callbacks ----------------------------- */

void TrajServer::replanCallback(std_msgs::Empty msg) {
  // Informed of new replan, end the current traj after some time
  const double time_out = 0.3;
  ros::Time time_now = ros::Time::now();
  double t_stop = (time_now - start_time_).toSec() + time_out + replan_time_;
  traj_duration_ = min(t_stop, traj_duration_);
}

void TrajServer::newCallback(std_msgs::Empty msg) {
  // Clear the executed traj data
  traj_cmd_.clear();
  traj_real_.clear();
  yaw_traj_cmd_.clear();
  yaw_traj_real_.clear();
}

void TrajServer::odomCallbck(const nav_msgs::Odometry &msg) {
  if (msg.child_frame_id == "X" || msg.child_frame_id == "O")
    return;
  odom = msg;
  traj_real_.push_back(Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y,
                                       odom.pose.pose.position.z));

  Eigen::Quaterniond odom_orient_(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
                                  odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
  Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
  double yaw_real = atan2(rot_x(1), rot_x(0));
  yaw_traj_real_.push_back(yaw_real);

  if (traj_real_.size() > 10000) {
    traj_real_.erase(traj_real_.begin(), traj_real_.begin() + 1000);
    yaw_traj_real_.erase(yaw_traj_real_.begin(), yaw_traj_real_.begin() + 1000);
  }
}

void TrajServer::visCallback(const ros::TimerEvent &e) {
  // Draw the executed traj (desired state)
  displayTrajWithColor(traj_cmd_, 0.05, Eigen::Vector4d(0, 0, 1, 1), pub_traj_id_);
}

void TrajServer::bsplineCallback(const trajectory::BsplineConstPtr &msg) {
  // Received traj should have ascending traj_id
  if (msg->traj_id <= traj_id_) {
    ROS_ERROR("out of order bspline.");
    ROS_ERROR("msg->traj_id: %d", msg->traj_id);
    ROS_ERROR("traj_id_: %d", traj_id_);
    return;
  }

  // Parse the msg
  Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);
  Eigen::VectorXd knots(msg->knots.size());
  for (int i = 0; i < msg->knots.size(); ++i) {
    knots(i) = msg->knots[i];
  }
  for (int i = 0; i < msg->pos_pts.size(); ++i) {
    pos_pts(i, 0) = msg->pos_pts[i].x;
    pos_pts(i, 1) = msg->pos_pts[i].y;
    pos_pts(i, 2) = msg->pos_pts[i].z;
  }
  NonUniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  for (int i = 0; i < msg->yaw_pts.size(); ++i)
    yaw_pts(i, 0) = msg->yaw_pts[i];
  NonUniformBspline yaw_traj(yaw_pts, 3, msg->yaw_dt);
  // start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());
  traj_.push_back(yaw_traj);
  traj_.push_back(yaw_traj.getDerivative());
  traj_.push_back(traj_[2].getDerivative());
  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;

  double yaw_cmd_start = traj_[3].evaluateDeBoorT(0)[0];
  double yaw_current = yaw_traj_real_.back();
  if (fabs(yaw_cmd_start - yaw_current) > 1e-3) {
    Eigen::Vector3d pos = traj_real_.back();
    prepareYaw(pos, yaw_cmd_start, yaw_current);
  }

  start_time_ = ros::Time::now();
  start_eval_ = true;
  start_eval_time_ = ros::Time::now().toSec();

  // Call vins service for start count features trigger
  // std_srvs::Empty srv;
  // if (start_cnt_client_.call(srv)) {
  //   ROS_INFO("Service call successful");
  // } else {
  //   ROS_ERROR("Failed to call service");
  // }

  // Record the start time of flight
  if (start_time.isZero()) {
    ROS_WARN("start flight");
    start_time = ros::Time::now();
  }
}

void TrajServer::cmdCallback(const ros::TimerEvent &e) {
  // No publishing before receive traj data
  if (!receive_traj_)
    return;

  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - start_time_).toSec();
  Eigen::Vector3d pos, vel, acc, jer;
  double yaw, yawdot;

  if (t_cur < traj_duration_ && t_cur >= 0.0) {
    // Current time within range of planned traj
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);
    yaw = traj_[3].evaluateDeBoorT(t_cur)[0];
    yawdot = traj_[4].evaluateDeBoorT(t_cur)[0];
    jer = traj_[5].evaluateDeBoorT(t_cur);

    cur_vel = vel.norm();
    cur_acc = acc.norm();
  } else if (t_cur >= traj_duration_) {
    // Call vins service for end count features trigger
    // static bool service_called = false;
    // std_srvs::Empty srv;
    // if (!service_called) {
    //   if (end_cnt_client_.call(srv)) {
    //     ROS_INFO("Service call successful");
    //     service_called = true;
    //   } else {
    //     ROS_ERROR("Failed to call service");
    //   }
    // }

    // Current time exceed range of planned traj
    // keep publishing the final position and yaw
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();
    yaw = traj_[3].evaluateDeBoorT(traj_duration_)[0];
    yawdot = 0.0;

    // Report info of the whole flight
    double len = calcPathLength(traj_cmd_);
    double flight_t = (end_time - start_time).toSec();
    ROS_WARN_ONCE("[Traj server] Estimation error: %.2lf, RMSE: %.2lf, Flight time: %.2lf, Path "
                  "length: %.2lf, Max vel: %.2lf, Max acc: %.2lf",
                  last_eval_error_, RMSE_, flight_t, len, max_vel, max_acc);

    // Write the evaluation result to txt
    // bool write_cumulative = false;
    // if (start_eval_ && write_cumulative) {
    //   std::ofstream ofs("/home/cindy/Downloads/icra2024/ours/eval_v2.0.txt", std::ios::app);
    //   if (!ofs.is_open()) {
    //     std::cerr << "Failed to open file: eval_cumulative.txt" << std::endl;
    //     return;
    //   }
    //   ofs << "Estimation error: " << last_eval_error_ << ", RMSE: " << RMSE_
    //       << ", Max vel: " << max_vel << ", Path length: " << len << std::endl;
    //   ofs.close();
    // }

    start_eval_ = false;

  } else {
    ROS_ERROR("[Traj server] Invalid time: start_time_: %.2lf, t_cur: %.2lf, traj_duration_: %.2lf",
              start_time_.toSec(), t_cur, traj_duration_);
  }

  // if (isLoopCorrection) {
  //   pos = R_loop.transpose() * (pos - T_loop);
  //   vel = R_loop.transpose() * vel;
  //   acc = R_loop.transpose() * acc;

  //   Eigen::Vector3d yaw_dir(cos(yaw), sin(yaw), 0);
  //   yaw_dir = R_loop.transpose() * yaw_dir;
  //   yaw = atan2(yaw_dir[1], yaw_dir[0]);
  // }

  if (vel.norm() > max_vel)
    max_vel = vel.norm();
  if (acc.norm() > max_acc)
    max_acc = acc.norm();

  cmd.header.stamp = time_now;
  cmd.trajectory_id = traj_id_;
  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);
  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);
  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);
  cmd.yaw = yaw;
  cmd.yaw_dot = yawdot;
  pos_cmd_pub.publish(cmd);

  // Draw cmd
  Eigen::Vector3d dir(cos(yaw), sin(yaw), 0.0);
  drawCmd(pos, 2 * dir, 2, Eigen::Vector4d(1, 1, 0, 0.7));
  drawCmd(pos, vel, 0, Eigen::Vector4d(0, 1, 0, 1));
  // drawCmd(pos, acc, 1, Eigen::Vector4d(0, 0, 1, 1));
  // drawCmd(pos, pos_err, 3, Eigen::Vector4d(1, 1, 0, 0.7));
  percep_utils_->setPose(pos, yaw);
  vector<Eigen::Vector3d> l1, l2;
  percep_utils_->getFOV(l1, l2);
  drawFOV(l1, l2);

  // Record info of the executed traj
  if (traj_cmd_.size() == 0) {
    // Add the first position
    traj_cmd_.push_back(pos);
    yaw_traj_cmd_.push_back(yaw);
  } else if ((pos - traj_cmd_.back()).norm() > 1e-6) {
    // Add new different commanded position
    // Detect pos jump, disabled for agile flight
    // Eigen::Vector3d last_pos = traj_cmd_.back();
    // if ((pos - last_pos).norm() > 0.05) {
    //   ROS_ERROR("[Traj server] pos jump from %.2lf, %.2lf, %.2lf to %.2lf, %.2lf, %.2lf",
    //             last_pos[0], last_pos[1], last_pos[2], pos[0], pos[1], pos[2]);
    // }

    traj_cmd_.push_back(pos);
    yaw_traj_cmd_.push_back(yaw);

    double dt = (time_now - last_time).toSec();
    energy += jer.squaredNorm() * dt;
    end_time = ros::Time::now();
  }
  last_time = time_now;

  // if (traj_cmd_.size() > 100000)
  //   traj_cmd_.erase(traj_cmd_.begin(), traj_cmd_.begin() + 1000);
}

void TrajServer::evaluateCallback(const nav_msgs::OdometryConstPtr &airsim_msg,
                                  const nav_msgs::OdometryConstPtr &vins_msg) {
  if (!start_eval_)
    return;

  // const double z_offset = 0.57;

  double time_now = ros::Time::now().toSec() - start_time.toSec();
  double time_diff = time_now - last_eval_time_;

  if (time_diff > 0.1) {
    last_eval_time_ = time_now;

    // Eigen::Vector3d airsim_pos(airsim_msg->pose.pose.position.x,
    // airsim_msg->pose.pose.position.y,
    //                            airsim_msg->pose.pose.position.z + z_offset);
    Eigen::Vector3d airsim_pos(airsim_msg->pose.pose.position.x, airsim_msg->pose.pose.position.y,
                               airsim_msg->pose.pose.position.z);
    Eigen::Vector3d vins_pos(vins_msg->pose.pose.position.x, vins_msg->pose.pose.position.y,
                             vins_msg->pose.pose.position.z);

    if (error_bias_.isZero())
      error_bias_ = airsim_pos - vins_pos;

    Eigen::Vector3d vins_aligned = vins_pos + error_bias_;
    double error = (airsim_pos - vins_aligned).norm();
    RMSE_ = sqrt((RMSE_ * RMSE_ * num_eval_ + error * error) / (num_eval_ + 1));

    num_eval_++;
    last_eval_error_ = error;

    ROS_INFO("[Traj server] Estimation error: %f, RMSE: %f", last_eval_error_, RMSE_);

    // Write the error for plot
    // bool write_plot = false;
    // if (write_plot) {
    //   double cur_t = ros::Time::now().toSec() - start_eval_time_;
    //   std::ofstream ofs("/home/cindy/Downloads/icra2024/ours/error_plot_ours.txt",
    //                     std::ios::app);
    //   if (!ofs.is_open()) {
    //     std::cerr << "Failed to open file: error_plot_ours.txt" << std::endl;
    //     return;
    //   }
    //   ofs << cur_t << "," << last_eval_error_ << std::endl;
    //   ofs.close();
    // }

    // // Write eval result into txt file
    // double write_to_file = false;
    // if (write_to_file) {
    //   if (time_now < 0.1) {
    //     std::ofstream ofs("/home/cindy/workspace/Agile_Perception_Aware_ws/eval.txt");
    //     ofs.close();
    //   }
    //   std::ofstream ofs("/home/cindy/workspace/Agile_Perception_Aware_ws/eval.txt", std::ios::app);
    //   if (!ofs.is_open()) {
    //     std::cerr << "Failed to open file: eval.txt" << std::endl;
    //     return;
    //   }
    //   double len = calcPathLength(traj_cmd_);
    //   // ofs << time_now << "," << last_eval_error_ << std::endl;
    //   ofs << len << "," << last_eval_error_ << std::endl;
    //   // ofs << len << "," << cur_vel << std::endl;
    //   ofs.close();
    // }
  }
}

bool TrajServer::startEvalCallback(std_srvs::Empty::Request &request,
                                   std_srvs::Empty::Response &response) {
  start_eval_ = true;
  start_eval_time_ = ros::Time::now().toSec();
  return true;
}

/* ---------------------------- Helper functions ---------------------------- */

void TrajServer::startTrigger() {
  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  trigger_pub.publish(path_msg);
}

double TrajServer::calcPathLength(const vector<Eigen::Vector3d> &path) {
  if (path.empty())
    return 0;
  double len = 0.0;
  for (int i = 0; i < path.size() - 1; ++i) {
    len += (path[i + 1] - path[i]).norm();
  }
  return len;
}

void TrajServer::prepareYaw(Eigen::Vector3d pos, double yaw_cmd, double yaw_cur) {
  // Round yaw_cur to [-PI, PI]
  while (yaw_cur < -M_PI)
    yaw_cur += 2 * M_PI;
  while (yaw_cur > M_PI)
    yaw_cur -= 2 * M_PI;

  double diff = yaw_cmd - yaw_cur;
  if (fabs(diff) <= M_PI) {
    yaw_cmd = yaw_cur + diff;
  } else if (diff > M_PI) {
    yaw_cmd = yaw_cur + diff - 2 * M_PI;
  } else if (diff < -M_PI) {
    yaw_cmd = yaw_cur + diff + 2 * M_PI;
  }
  diff = yaw_cmd - yaw_cur;

  // Set b-spline params
  const int seg_num = 12;
  const double yaw_vel = M_PI / 12.0;
  const double duration = fabs(diff) / yaw_vel;
  const double dt_yaw = duration / seg_num; // time of B-spline segment

  // Define waypoints such that yaw change is uniform
  vector<Eigen::Vector3d> waypoints;
  for (int i = 0; i < seg_num + 1; i++) {
    double t = i * dt_yaw;
    double yaw = yaw_cur + diff * t / duration;
    Eigen::Vector3d waypt;
    waypt(0) = yaw;
    waypt(1) = waypt(2) = 0.0;
    waypoints.push_back(waypt);
  }

  Eigen::MatrixXd points(waypoints.size(), 3);
  for (int i = 0; i < waypoints.size(); ++i)
    points.row(i) = waypoints[i].transpose();

  Eigen::VectorXd times(waypoints.size() - 1);
  times.setConstant(dt_yaw);

  // Given desired waypoints and corresponding time stamps, fit a B-spline and execute it
  PolynomialTraj poly;
  PolynomialTraj::waypointsTraj(points, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), times, poly);

  // Fit the polynomial with B-spline
  vector<Eigen::Vector3d> point_set, boundary_der;
  for (double ts = 0; ts <= 1e-3 + duration; ts += dt_yaw)
    point_set.push_back(poly.evaluate(ts, 0));

  boundary_der.push_back(poly.evaluate(0, 1));
  boundary_der.push_back(poly.evaluate(duration, 1));
  boundary_der.push_back(poly.evaluate(0, 2));
  boundary_der.push_back(poly.evaluate(duration, 2));

  Eigen::MatrixXd yaw_ctrl_pts;
  NonUniformBspline::parameterizeToBspline(dt_yaw, point_set, boundary_der, 3, yaw_ctrl_pts);
  NonUniformBspline fitted(yaw_ctrl_pts, 3, dt_yaw);
  NonUniformBspline vel = fitted.getDerivative();

  ros::Duration(0.1).sleep();

  ROS_WARN("[Traj_server] Yaw preparation starts");

  auto t1 = ros::Time::now();
  double tn = (ros::Time::now() - t1).toSec();
  while (tn < duration && ros::ok()) {
    cmd.header.stamp = ros::Time::now();
    cmd.position.x = pos(0);
    cmd.position.y = pos(1);
    cmd.position.z = pos(2);
    cmd.velocity.x = 0;
    cmd.velocity.y = 0;
    cmd.velocity.z = 0;
    cmd.acceleration.x = 0;
    cmd.acceleration.y = 0;
    cmd.acceleration.z = 0;
    cmd.yaw = fitted.evaluateDeBoorT(tn)(0);
    cmd.yaw_dot = vel.evaluateDeBoorT(tn)(0);
    pos_cmd_pub.publish(cmd);

    ros::Duration(0.02).sleep();
    tn = (ros::Time::now() - t1).toSec();
  }

  ros::Duration(2.0).sleep();
  ROS_WARN("[Traj_server] Yaw preparation done");
}

/* ------------------------------ Visualization ----------------------------- */

void TrajServer::displayTrajWithColor(vector<Eigen::Vector3d> path, double resolution,
                                      Eigen::Vector4d color, int id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  traj_pub.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;
  geometry_msgs::Point pt;
  for (int i = 0; i < int(path.size()); i++) {
    pt.x = path[i](0);
    pt.y = path[i](1);
    pt.z = path[i](2);
    mk.points.push_back(pt);
  }
  traj_pub.publish(mk);
  ros::Duration(0.001).sleep();
}

void TrajServer::drawFOV(const vector<Eigen::Vector3d> &list1,
                         const vector<Eigen::Vector3d> &list2) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = 0;
  mk.ns = "current_pose";
  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.color.a = 1.0;
  mk.scale.x = 0.04;
  mk.scale.y = 0.04;
  mk.scale.z = 0.04;

  // Clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  cmd_vis_pub.publish(mk);

  if (list1.size() == 0)
    return;

  // Pub new marker
  geometry_msgs::Point pt;
  for (int i = 0; i < int(list1.size()); ++i) {
    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = list1[i](2);
    mk.points.push_back(pt);

    pt.x = list2[i](0);
    pt.y = list2[i](1);
    pt.z = list2[i](2);
    mk.points.push_back(pt);
  }
  mk.action = visualization_msgs::Marker::ADD;
  cmd_vis_pub.publish(mk);
}

void TrajServer::drawCmd(const Eigen::Vector3d &pos, const Eigen::Vector3d &vec, const int &id,
                         const Eigen::Vector4d &color) {
  visualization_msgs::Marker mk_state;
  mk_state.header.frame_id = "world";
  mk_state.header.stamp = ros::Time::now();
  mk_state.id = id;
  mk_state.type = visualization_msgs::Marker::ARROW;
  mk_state.action = visualization_msgs::Marker::ADD;

  mk_state.pose.orientation.w = 1.0;
  mk_state.scale.x = 0.1;
  mk_state.scale.y = 0.2;
  mk_state.scale.z = 0.3;

  geometry_msgs::Point pt;
  pt.x = pos(0);
  pt.y = pos(1);
  pt.z = pos(2);
  mk_state.points.push_back(pt);

  pt.x = pos(0) + vec(0);
  pt.y = pos(1) + vec(1);
  pt.z = pos(2) + vec(2);
  mk_state.points.push_back(pt);

  mk_state.color.r = color(0);
  mk_state.color.g = color(1);
  mk_state.color.b = color(2);
  mk_state.color.a = color(3);

  cmd_vis_pub.publish(mk_state);
}

/* ----------------------------- Test functions ----------------------------- */

void TrajServer::fitBsplineAndExecute(Eigen::MatrixXd &waypoints, Eigen::VectorXd &times) {
  // Given desired waypoints and corresponding time stamps, fit a B-spline and execute it
  PolynomialTraj poly;
  PolynomialTraj::waypointsTraj(waypoints, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), times, poly);

  const int degree = 5;
  double duration = poly.getTotalTime();
  vector<Eigen::Vector3d> traj_pts;
  for (double ts = 0; ts <= duration; ts += 0.01)
    traj_pts.push_back(poly.evaluate(ts, 0));
  // displayTrajWithColor(traj_pts, 0.05, Eigen::Vector4d(1, 0, 0, 1), 99);

  // Fit the polynomial with B-spline
  const int seg_num = 30;
  double dt = duration / seg_num;
  vector<Eigen::Vector3d> point_set, boundary_der;
  for (double ts = 0; ts <= 1e-3 + duration; ts += dt)
    point_set.push_back(poly.evaluate(ts, 0));

  boundary_der.push_back(poly.evaluate(0, 1));
  boundary_der.push_back(poly.evaluate(duration, 1));
  boundary_der.push_back(poly.evaluate(0, 2));
  boundary_der.push_back(poly.evaluate(duration, 2));

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(dt, point_set, boundary_der, degree, ctrl_pts);
  NonUniformBspline fitted(ctrl_pts, degree, dt);

  traj_pts.clear();
  double duration2 = fitted.getTimeSum();
  for (double ts = 0; ts <= duration2; ts += 0.01)
    traj_pts.push_back(fitted.evaluateDeBoorT(ts));

  vector<Eigen::Vector3d> ctrl_pts_vec;
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    Eigen::Vector3d pr = ctrl_pts.row(i).transpose();
    ctrl_pts_vec.push_back(pr);
  }
  displayTrajWithColor(ctrl_pts_vec, 0.1, Eigen::Vector4d(1, 1, 0, 1), 98);
  displayTrajWithColor(traj_pts, 0.05, Eigen::Vector4d(1, 0, 0, 1), 99);

  auto vel = fitted.getDerivative();
  auto acc = vel.getDerivative();

  ros::Duration(0.1).sleep();

  // Pub the traj
  ROS_WARN("[Traj_server] Publishing trajectory of duration %f, duration2 %f", duration, duration2);
  auto t1 = ros::Time::now();
  double tn = (ros::Time::now() - t1).toSec();
  while (tn < duration && ros::ok()) {
    // Eigen::Vector3d p = bspline.evaluateDeBoorT(tn);
    // Eigen::Vector3d v = vel.evaluateDeBoorT(tn);
    // Eigen::Vector3d a = acc.evaluateDeBoorT(tn);
    Eigen::Vector3d p = fitted.evaluateDeBoorT(tn);
    Eigen::Vector3d v = vel.evaluateDeBoorT(tn);
    Eigen::Vector3d a = acc.evaluateDeBoorT(tn);

    cmd.header.stamp = ros::Time::now();
    cmd.position.x = p(0);
    cmd.position.y = p(1);
    cmd.position.z = p(2);
    cmd.velocity.x = v(0);
    cmd.velocity.y = v(1);
    cmd.velocity.z = v(2);
    cmd.acceleration.x = a(0);
    cmd.acceleration.y = a(1);
    cmd.acceleration.z = a(2);
    pos_cmd_pub.publish(cmd);

    ros::Duration(0.02).sleep();
    tn = (ros::Time::now() - t1).toSec();
  }
}

void TrajServer::sin_curve_traj() {
  // Generate the first B-spline's control points from a sin curve
  vector<Eigen::Vector3d> samples;
  const double dt1 = M_PI / 6.0;
  for (double theta = 0; theta <= 2 * M_PI; theta += dt1) {
    Eigen::Vector3d sample(theta, sin(theta), 1);
    samples.push_back(sample);
  }
  Eigen::MatrixXd points(samples.size(), 3);
  for (int i = 0; i < samples.size(); ++i)
    points.row(i) = samples[i].transpose();

  Eigen::VectorXd times(samples.size() - 1);
  times.setConstant(dt1);
  times[0] += dt1;
  times[times.rows() - 1] += dt1;

  fitBsplineAndExecute(points, times);
}

void TrajServer::straight_line_traj() {
  vector<Eigen::Vector3d> samples;
  const double dt = 5;
  for (double theta = 0; theta <= 18; theta += dt) {
    Eigen::Vector3d sample(theta, 0, 1);
    samples.push_back(sample);
  }
  Eigen::MatrixXd points(samples.size(), 3);
  for (int i = 0; i < samples.size(); ++i)
    points.row(i) = samples[i].transpose();

  Eigen::VectorXd times(samples.size() - 1);
  times.setConstant(dt);
  times[0] += dt;
  times[times.rows() - 1] += dt;

  fitBsplineAndExecute(points, times);
}
} // namespace fast_planner