// #include <fstream>
#include <plan_manage/planner_manager.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <thread>
#include <visualization_msgs/Marker.h>

namespace fast_planner {
// SECTION interfaces for setup and query

FastPlannerManager::FastPlannerManager() {}

FastPlannerManager::~FastPlannerManager() {}

void FastPlannerManager::initPlanModules(ros::NodeHandle &nh) {
  nh.param("manager/max_vel", pp_.max_vel_, -1.0);
  nh.param("manager/max_acc", pp_.max_acc_, -1.0);
  nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
  nh.param("manager/accept_vel", pp_.accept_vel_, pp_.max_vel_ + 0.5);
  nh.param("manager/accept_acc", pp_.accept_acc_, pp_.max_acc_ + 0.5);
  nh.param("manager/max_yawdot", pp_.max_yawdot_, -1.0);
  nh.param("manager/clearance_threshold", pp_.clearance_, -1.0);
  nh.param("manager/local_segment_length", pp_.local_traj_len_, -1.0);
  nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
  nh.param("manager/bspline_degree", pp_.bspline_degree_, 3);
  nh.param("manager/min_time", pp_.min_time_, false);

  bool use_geometric_path, use_optimization, use_initial_yaw, use_parallax;
  nh.param("manager/use_geometric_path", use_geometric_path, false);
  nh.param("manager/use_optimization", use_optimization, false);
  nh.param("manager/use_initial_yaw", use_initial_yaw, false);
  nh.param("manager/use_parallax", use_parallax, false);

  nh.param("manager/occupancy_map_file", occupancy_map_file_, string(""));
  nh.param("manager/esdf_map_file", esdf_map_file_, string(""));
  nh.param("manager/feature_map_file", feature_map_file_, string(""));

  local_data_.traj_id_ = 0;
  map_server_.reset(new voxel_mapping::MapServer(nh));
  edt_environment_.reset(new EDTEnvironment);
  edt_environment_->setMap(map_server_);

  double resolution = edt_environment_->map_server_->getResolution();
  Eigen::Vector3d origin, size;
  edt_environment_->map_server_->getRegion(origin, size);
  caster_.reset(new RayCaster);
  caster_->setParams(resolution, origin);

  if (use_geometric_path) {
    path_finder_.reset(new Astar);
    // path_finder_->setParam(nh);
    // path_finder_->setEnvironment(edt_environment_);
    // path_finder_->init();
    path_finder_->init(nh, edt_environment_);
  }

  if (use_optimization) {
    bspline_optimizers_.resize(10);
    for (int i = 0; i < 10; ++i) {
      bspline_optimizers_[i].reset(new BsplineOptimizer);
      bspline_optimizers_[i]->setParam(nh);
      bspline_optimizers_[i]->setEnvironment(edt_environment_);
    }
  }

  if (use_initial_yaw) {
    yaw_initial_planner_.reset(new YawInitialPlanner(nh));
    yaw_initial_planner_->setMap(map_server_);
  }

  if (use_parallax) {
    for (int i = 0; i < 10; ++i) {
      bspline_optimizers_[i]->initParallaxUtil(nh);
    }
  }
}

void FastPlannerManager::saveMapService() { map_server_->saveMap(occupancy_map_file_); }

void FastPlannerManager::loadMapService() {
  map_server_->loadMap(occupancy_map_file_, esdf_map_file_, feature_map_file_);
}

void FastPlannerManager::setGlobalWaypoints(vector<Eigen::Vector3d> &waypoints) {
  plan_data_.global_waypoints_ = waypoints;
}

bool FastPlannerManager::checkTrajCollision(double &distance) {
  double t_now = (ros::Time::now() - local_data_.start_time_).toSec();

  Eigen::Vector3d cur_pt = local_data_.position_traj_.evaluateDeBoorT(t_now);
  double radius = 0.0;
  Eigen::Vector3d fut_pt;
  double fut_t = 0.02;

  while (radius < 6.0 && t_now + fut_t < local_data_.duration_) {
    fut_pt = local_data_.position_traj_.evaluateDeBoorT(t_now + fut_t);
    // double dist = edt_environment_->map_server_->getDistance(fut_pt);
    if (map_server_->getOccupancy(fut_pt) == voxel_mapping::OccupancyType::OCCUPIED) {
      distance = radius;
      // std::cout << "collision at: " << fut_pt.transpose() << ", dist: " <<
      // dist << std::endl;
      std::cout << "collision at: " << fut_pt.transpose() << std::endl;
      return false;
    }
    radius = (fut_pt - cur_pt).norm();
    fut_t += 0.02;
  }

  return true;
}

// !SECTION

void FastPlannerManager::planExploreTraj(const vector<Eigen::Vector3d> &tour,
                                         const Eigen::Vector3d &cur_vel,
                                         const Eigen::Vector3d &cur_acc, const bool reach_end,
                                         const double &time_lb) {
  if (tour.empty())
    ROS_ERROR("Empty path to traj planner");

  // Generate traj through waypoints-based method
  const int pt_num = tour.size();
  Eigen::MatrixXd pos(pt_num, 3);
  for (int i = 0; i < pt_num; ++i) {
    pos.row(i) = tour[i];
  }

  Eigen::Vector3d zero(0, 0, 0);
  Eigen::VectorXd times(pt_num - 1);
  for (int i = 0; i < pt_num - 1; ++i)
    times(i) = (pos.row(i + 1) - pos.row(i)).norm() / (pp_.max_vel_ * 0.5);

  PolynomialTraj init_traj;
  PolynomialTraj::waypointsTraj(pos, cur_vel, zero, cur_acc, zero, times, init_traj);

  // B-spline-based optimization
  vector<Vector3d> points, boundary_deri;
  double duration = init_traj.getTotalTime();
  int seg_num = init_traj.getLength() / pp_.ctrl_pt_dist;
  seg_num = max(8, seg_num);
  double dt = duration / double(seg_num);

  // std::cout << "duration: " << duration << ", seg_num: " << seg_num << ", dt: " << dt <<
  // std::endl;

  for (double ts = 0.0; ts <= duration + 1e-4; ts += dt)
    points.push_back(init_traj.evaluate(ts, 0));
  // Evaluate velocity at start and end
  boundary_deri.push_back(init_traj.evaluate(0.0, 1));
  boundary_deri.push_back(init_traj.evaluate(duration, 1));
  // Evaluate acceleration at start and end
  boundary_deri.push_back(init_traj.evaluate(0.0, 2));
  boundary_deri.push_back(init_traj.evaluate(duration, 2));

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(dt, points, boundary_deri, pp_.bspline_degree_,
                                           ctrl_pts);
  NonUniformBspline tmp_traj(ctrl_pts, pp_.bspline_degree_, dt);

  int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::FEASIBILITY |
                  BsplineOptimizer::START | BsplineOptimizer::END | BsplineOptimizer::MINTIME |
                  BsplineOptimizer::DISTANCE | BsplineOptimizer::PARALLAX |
                  BsplineOptimizer::VERTICALVISIBILITY;
  // int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::FEASIBILITY |
  //                 BsplineOptimizer::START | BsplineOptimizer::END | BsplineOptimizer::MINTIME |
  //                 BsplineOptimizer::DISTANCE;

  vector<Vector3d> start, end;
  vector<bool> start_idx, end_idx;

  if (reach_end) {
    tmp_traj.getBoundaryStates(2, 2, start, end);
    start_idx = {true, true, true};
    end_idx = {true, true, true};
  } else {
    tmp_traj.getBoundaryStates(2, 0, start, end);
    start_idx = {true, true, true};
    end_idx = {true, false, false};
  }

  bspline_optimizers_[0]->setBoundaryStates(start, end, start_idx, end_idx);
  if (time_lb > 0)
    bspline_optimizers_[0]->setTimeLowerBound(time_lb);

  // bspline_optimizers_[0]->optimize(ctrl_pts, dt, cost_func, 1, 1);
  bspline_optimizers_[0]->optimize(ctrl_pts, dt, cost_func, 3, 3);
  local_data_.position_traj_.setUniformBspline(ctrl_pts, pp_.bspline_degree_, dt);

  // double t_total = (ros::Time::now() - t1).toSec();
  // ROS_WARN("[planExploreTraj] opt: %lf s", t_total);

  updateTrajInfo();

  vector<Eigen::Vector3d> pos_knot_points;
  local_data_.position_traj_.getKnotPoint(pos_knot_points);
}

int FastPlannerManager::planLocalMotion(const Vector3d &next_pos, const Vector3d &pos,
                                        const Vector3d &vel, const Vector3d &acc, bool &truncated,
                                        const double &time_lb) {
  // Start optimization
  // Plan trajectory (position and yaw) to the next viewpoint

  // Generate trajectory of x,y,z
  path_finder_->reset();
  if (path_finder_->search(pos, next_pos) != Astar::REACH_END) {
    ROS_ERROR("No path to point (%f, %f, %f)", next_pos.x(), next_pos.y(), next_pos.z());
    std::cout << "pos:" << pos.transpose() << std::endl;
    std::cout << "next_pos:" << next_pos.transpose() << std::endl;
    return LOCAL_FAIL;
  }

  // ROS_INFO("[planLocalMotion] Start p: %.2f, %.2f, %.2f, v: %.2f, %.2f, %.2f, a: %.2f, %.2f,
  // %.2f",
  //          start_pt.x(), start_pt.y(), start_pt.z(), start_vel.x(), start_vel.y(), start_vel.z(),
  //          start_acc.x(), start_acc.y(), start_acc.z());
  // ROS_INFO("[planLocalMotion] Goal p: %.2f, %.2f, %.2f, v: %.2f, %.2f, %.2f",
  //          end_pt.x(), end_pt.y(), end_pt.z(), end_vel.x(), end_vel.y(), end_vel.z());

  vector<Vector3d> path_waypoint = path_finder_->getPath();
  shortenPath(path_waypoint);
  vector<Eigen::Vector3d> truncated_path;

  // const double radius_far = 7.0;
  const double radius_far = 50.0;
  const double radius_close = 1.5;
  const double len = Astar::pathLength(path_waypoint);

  // Next viewpoint is far away, truncate the far goal to an intermediate goal
  if (len > radius_far) {
    truncated = true;
    truncated_path.push_back(path_waypoint.front());
    double len2 = 0.0;
    for (int i = 1; i < path_waypoint.size() && len2 < radius_far; ++i) {
      auto cur_pt = path_waypoint[i];
      len2 += (cur_pt - truncated_path.back()).norm();
      truncated_path.push_back(cur_pt);
    }
  }

  vector<Eigen::Vector3d> input_path = truncated ? truncated_path : path_waypoint;
  planExploreTraj(input_path, vel, acc, !truncated, time_lb);

  return LOCAL_SUCCEED;
}

void FastPlannerManager::shortenPath(vector<Vector3d> &path) {
  if (path.empty()) {
    ROS_ERROR("Empty path to shorten");
    return;
  }
  // Shorten the tour, only critical intermediate points are reserved.
  // See Fig.8 from "Robust Real-time UAV Replanning Using Guided Gradient-based
  // Optimization and Topological Paths"
  const double dist_thresh = 3.0;
  vector<Vector3d> short_tour = {path.front()};
  for (int i = 1; i < path.size() - 1; ++i) {
    if ((path[i] - short_tour.back()).norm() > dist_thresh)
      short_tour.push_back(path[i]);
    else {
      // Add waypoints to shorten path only to avoid collision
      caster_->input(short_tour.back(), path[i + 1]);
      Eigen::Vector3i idx;
      while (caster_->nextId(idx) && ros::ok()) {
        if (edt_environment_->map_server_->getOccupancy(idx) ==
                voxel_mapping::OccupancyType::OCCUPIED ||
            edt_environment_->map_server_->getOccupancy(idx) ==
                voxel_mapping::OccupancyType::UNKNOWN) {
          short_tour.push_back(path[i]);
          break;
        }
      }
    }
  }
  if ((path.back() - short_tour.back()).norm() > 1e-3)
    short_tour.push_back(path.back());

  // Ensure at least three points in the path
  if (short_tour.size() == 2)
    short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));
  path = short_tour;
}

void FastPlannerManager::selectBestTraj(NonUniformBspline &traj) {
  // sort by jerk
  vector<NonUniformBspline> &trajs = plan_data_.topo_traj_pos2_;
  sort(trajs.begin(), trajs.end(), [](NonUniformBspline &tj1, NonUniformBspline &tj2) {
    return tj1.getJerk() < tj2.getJerk();
  });
  traj = trajs[0];
}

void FastPlannerManager::refineTraj(NonUniformBspline &best_traj) {
  ros::Time t1 = ros::Time::now();
  plan_data_.no_visib_traj_ = best_traj;

  int cost_function = BsplineOptimizer::NORMAL_PHASE;
  if (pp_.min_time_)
    cost_function |= BsplineOptimizer::MINTIME;

  // ViewConstraint view_cons;
  // visib_util_->calcViewConstraint(best_traj, view_cons);
  // plan_data_.view_cons_ = view_cons;
  // if (view_cons.idx_ >= 0)
  // {
  //   cost_function |= BsplineOptimizer::VIEWCONS;
  //   bspline_optimizers_[0]->setViewConstraint(view_cons);
  // }

  // Refine selected best traj
  Eigen::MatrixXd ctrl_pts = best_traj.getControlPoint();
  double dt = best_traj.getKnotSpan();
  vector<Eigen::Vector3d> start1, end1;
  best_traj.getBoundaryStates(2, 0, start1, end1);
  vector<bool> start_idx = {true, true, true};
  vector<bool> end_idx = {true, false, false};

  bspline_optimizers_[0]->setBoundaryStates(start1, end1, start_idx, end_idx);
  bspline_optimizers_[0]->optimize(ctrl_pts, dt, cost_function, 2, 2);
  best_traj.setUniformBspline(ctrl_pts, pp_.bspline_degree_, dt);

  vector<Eigen::Vector3d> start2, end2;
  best_traj.getBoundaryStates(2, 2, start2, end2);
  for (int i = 0; i < 3; ++i)
    std::cout << "error start: " << (start1[i] - start2[i]).norm() << std::endl;
  for (int i = 0; i < 1; ++i)
    std::cout << "error end  : " << (end1[i] - end2[i]).norm() << std::endl;
}

void FastPlannerManager::updateTrajInfo() {
  local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
  local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();

  local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
  local_data_.duration_ = local_data_.position_traj_.getTimeSum();

  local_data_.traj_id_ += 1;
}

void FastPlannerManager::reparamBspline(NonUniformBspline &bspline, double ratio,
                                        Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc) {
  int prev_num = bspline.getControlPoint().rows();
  double time_origin = bspline.getTimeSum();

  int seg_num = bspline.getControlPoint().rows() - pp_.bspline_degree_;
  ratio = min(1.01, ratio);

  bspline.lengthenTime(ratio);
  double duration = bspline.getTimeSum();
  dt = duration / double(seg_num);
  time_inc = duration - time_origin;

  vector<Eigen::Vector3d> point_set;
  for (double time = 0.0; time <= duration + 1e-4; time += dt)
    point_set.push_back(bspline.evaluateDeBoorT(time));
  NonUniformBspline::parameterizeToBspline(dt, point_set, plan_data_.local_start_end_derivative_,
                                           pp_.bspline_degree_, ctrl_pts);
  // ROS_WARN("prev: %d, new: %d", prev_num, ctrl_pts.rows());
}

Eigen::MatrixXd FastPlannerManager::paramLocalTraj(double start_t, double &dt, double &duration) {
  vector<Eigen::Vector3d> point_set;
  vector<Eigen::Vector3d> start_end_derivative;
  global_data_.getTrajInfoInSphere(start_t, pp_.local_traj_len_, pp_.ctrl_pt_dist, point_set,
                                   start_end_derivative, dt, duration);

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, pp_.bspline_degree_,
                                           ctrl_pts);
  plan_data_.local_start_end_derivative_ = start_end_derivative;

  return ctrl_pts;
}

Eigen::MatrixXd FastPlannerManager::reparamLocalTraj(const double &start_t, const double &duration,
                                                     const double &dt) {
  vector<Eigen::Vector3d> point_set;
  vector<Eigen::Vector3d> start_end_derivative;

  global_data_.getTrajInfoInDuration(start_t, duration, dt, point_set, start_end_derivative);
  plan_data_.local_start_end_derivative_ = start_end_derivative;

  /* parameterization of B-spline */
  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, pp_.bspline_degree_,
                                           ctrl_pts);
  // cout << "ctrl pts:" << ctrl_pts.rows() << endl;

  return ctrl_pts;
}

void FastPlannerManager::planYaw(const Eigen::Vector3d &start_yaw) {
  auto t1 = ros::Time::now();
  // calculate waypoints of heading

  auto &pos = local_data_.position_traj_;
  double duration = pos.getTimeSum();

  double dt_yaw = 0.3;
  int seg_num = ceil(duration / dt_yaw);
  dt_yaw = duration / seg_num;

  const double forward_t = 2.0;
  double last_yaw = start_yaw(0);
  vector<Eigen::Vector3d> waypts;
  vector<int> waypt_idx;

  // seg_num -> seg_num - 1 points for constraint excluding the boundary states

  for (int i = 0; i < seg_num; ++i) {
    double tc = i * dt_yaw;
    Eigen::Vector3d pc = pos.evaluateDeBoorT(tc);
    double tf = min(duration, tc + forward_t);
    Eigen::Vector3d pf = pos.evaluateDeBoorT(tf);
    Eigen::Vector3d pd = pf - pc;

    Eigen::Vector3d waypt;
    if (pd.norm() > 1e-6) {
      waypt(0) = atan2(pd(1), pd(0));
      waypt(1) = waypt(2) = 0.0;
      calcNextYaw(last_yaw, waypt(0));
    } else {
      waypt = waypts.back();
    }
    last_yaw = waypt(0);
    waypts.push_back(waypt);
    waypt_idx.push_back(i);
  }

  // calculate initial control points with boundary state constraints

  Eigen::MatrixXd yaw(seg_num + 3, 1);
  yaw.setZero();

  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw,
      1.0, dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;
  yaw.block(0, 0, 3, 1) = states2pts * start_yaw;

  Eigen::Vector3d end_v = local_data_.velocity_traj_.evaluateDeBoorT(duration - 0.1);
  Eigen::Vector3d end_yaw(atan2(end_v(1), end_v(0)), 0, 0);
  calcNextYaw(last_yaw, end_yaw(0));
  yaw.block(seg_num, 0, 3, 1) = states2pts * end_yaw;

  // solve
  bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);
  int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::WAYPOINTS |
                  BsplineOptimizer::START | BsplineOptimizer::END;

  vector<Eigen::Vector3d> start = {Eigen::Vector3d(start_yaw[0], 0, 0),
                                   Eigen::Vector3d(start_yaw[1], 0, 0),
                                   Eigen::Vector3d(start_yaw[2], 0, 0)};
  vector<Eigen::Vector3d> end = {Eigen::Vector3d(end_yaw[0], 0, 0),
                                 Eigen::Vector3d(end_yaw[1], 0, 0),
                                 Eigen::Vector3d(end_yaw[2], 0, 0)};
  vector<bool> start_idx = {true, true, true};
  vector<bool> end_idx = {true, true, true};

  bspline_optimizers_[1]->setBoundaryStates(start, end, start_idx, end_idx);
  std::cout << "yaw: " << yaw << std::endl;
  bspline_optimizers_[1]->optimize(yaw, dt_yaw, cost_func, 1, 1);

  // update traj info
  local_data_.yaw_traj_.setUniformBspline(yaw, pp_.bspline_degree_, dt_yaw);
  local_data_.yawdot_traj_ = local_data_.yaw_traj_.getDerivative();
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();

  vector<double> path_yaw;
  for (int i = 0; i < waypts.size(); ++i)
    path_yaw.push_back(waypts[i][0]);
  plan_data_.path_yaw_ = path_yaw;
  plan_data_.dt_yaw_ = dt_yaw;
  plan_data_.dt_yaw_path_ = dt_yaw;

  std::cout << "yaw time: " << (ros::Time::now() - t1).toSec() << std::endl;
}

void FastPlannerManager::planYawPercepAgnostic() {
  // Yaw b-spline has same segment number as position b-spline
  Eigen::MatrixXd position_ctrl_pts = local_data_.position_traj_.getControlPoint();
  int ctrl_pts_num = position_ctrl_pts.rows();
  double dt_yaw = local_data_.position_traj_.getKnotSpan();

  // Yaw traj control points
  Eigen::MatrixXd yaw(ctrl_pts_num, 1);
  yaw.setZero();

  // Compute look-forwards waypoints
  vector<Eigen::Vector3d> waypts;
  vector<int> waypt_idx;
  const double forward_t = 2.0;
  double last_yaw = 0.0;
  bool first_last_yaw_flag = false;
  Eigen::VectorXd knots = local_data_.position_traj_.getKnot();
  vector<Eigen::Vector3d> knot_pts;
  local_data_.position_traj_.getKnotPoint(knot_pts);

  for (int i = 0; i < knot_pts.size() - 1; ++i) {
    double uc = knots(i);
    Eigen::Vector3d pc = local_data_.position_traj_.evaluateDeBoor(uc);
    double uf = min(local_data_.duration_, uc + forward_t);
    Eigen::Vector3d pf = local_data_.position_traj_.evaluateDeBoor(uf);
    // Eigen::Vector3d pc = knot_pts[i];
    // Eigen::Vector3d pf = knot_pts[i + 1];
    Eigen::Vector3d pd = pf - pc;
    Eigen::Vector3d waypt;
    if (pd.norm() > 1e-6) {
      waypt(0) = atan2(pd(1), pd(0));
      waypt(1) = waypt(2) = 0.0;
      if (!first_last_yaw_flag) {
        last_yaw = waypt(0);
        first_last_yaw_flag = true;
      } else
        calcNextYaw(last_yaw, waypt(0));
    } else
      waypt = waypts.back();

    last_yaw = waypt(0);
    waypts.push_back(waypt);
    waypt_idx.push_back(i);
  }

  // Initial state and final state
  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw,
      1.0, dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;

  yaw.block<3, 1>(0, 0) = states2pts * waypts.front();
  yaw.block<3, 1>(ctrl_pts_num - 3, 0) = states2pts * waypts.back();

  // Call B-spline optimization solver
  int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::START | BsplineOptimizer::END |
                  BsplineOptimizer::WAYPOINTS;

  Eigen::Vector3d unspecified = Eigen::Vector3d::Zero();
  vector<Eigen::Vector3d> start = {unspecified, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  vector<Eigen::Vector3d> end = {unspecified, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  vector<bool> start_idx = {false, true, true};
  vector<bool> end_idx = {false, true, true};

  bspline_optimizers_[1]->setBoundaryStates(start, end, start_idx, end_idx);
  bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);
  bspline_optimizers_[1]->optimize(yaw, dt_yaw, cost_func, 1, 1);

  // Update traj info
  local_data_.yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
  local_data_.yawdot_traj_ = local_data_.yaw_traj_.getDerivative();
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();
  plan_data_.dt_yaw_ = dt_yaw;
}

void FastPlannerManager::planYawCovisibility() {
  // Yaw b-spline has same segment number as position b-spline
  Eigen::MatrixXd position_ctrl_pts = local_data_.position_traj_.getControlPoint();
  int ctrl_pts_num = position_ctrl_pts.rows();
  double dt_yaw = local_data_.position_traj_.getKnotSpan();

  // Yaw traj control points
  Eigen::MatrixXd yaw(ctrl_pts_num, 1);
  yaw.setZero();

  // Calculate knot pos and acc
  // [u[p],u[m-p]] -> [0*dt, (m-2p)*dt] -> [0*dt, (n-2)*dt]
  vector<Eigen::Vector3d> twb_pos, twb_acc;
  for (int i = 0; i < ctrl_pts_num - 2; ++i) {
    double t = i * dt_yaw;
    Eigen::Vector3d pos = local_data_.position_traj_.evaluateDeBoorT(t);
    Eigen::Vector3d acc = local_data_.acceleration_traj_.evaluateDeBoorT(t);
    twb_pos.push_back(pos);
    twb_acc.push_back(acc);
    // cout << "pos: " << pos.transpose() << endl;
  }

  // TODO: only need to calculate nn features once! Feed to yaw_initial_planner & optimizer

  // Yaw initial planner
  vector<double> yaw_waypoints;
  yaw_initial_planner_->searchPathOfYaw(twb_pos, twb_acc, dt_yaw, yaw_waypoints);
  // std::cout << "yaw_waypoints: ";
  // for (int i = 0; i < yaw_waypoints.size(); ++i)
  //   std::cout << yaw_waypoints[i] << ", ";
  // std::cout << std::endl;

  // Set waypoints
  vector<Eigen::Vector3d> waypts;
  vector<int> waypt_idx;
  double last_yaw = yaw_waypoints[0];
  for (int i = 0; i < yaw_waypoints.size(); ++i) {
    Eigen::Vector3d waypt;
    waypt(0) = yaw_waypoints[i];
    waypt(1) = waypt(2) = 0.0;
    calcNextYaw(last_yaw, waypt(0));
    last_yaw = waypt(0);
    waypts.push_back(waypt);
    waypt_idx.push_back(i);
  }

  // Initial state
  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw,
      1.0, dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;
  Eigen::Vector3d start_yaw3d = waypts[0];
  yaw.block<3, 1>(0, 0) = states2pts * start_yaw3d;
  Eigen::Vector3d unspecified = Eigen::Vector3d::Zero();
  vector<Eigen::Vector3d> start = {unspecified, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  vector<Eigen::Vector3d> end = {unspecified, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  vector<bool> start_idx = {false, true, true};
  vector<bool> end_idx = {false, true, true};

  auto t1 = ros::Time::now();

  // Call B-spline optimization solver
  // BsplineOptimizer::YAWFEASIBILITY
  int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::WAYPOINTS |
                  BsplineOptimizer::START | BsplineOptimizer::END |
                  BsplineOptimizer::YAWCOVISIBILITY;

  if (cost_func & BsplineOptimizer::YAWCOVISIBILITY) {
    vector<Eigen::Vector3d> pos_knots, acc_knots;
    local_data_.position_traj_.getKnotPoint(pos_knots);
    local_data_.acceleration_traj_.getKnotPoint(acc_knots);
    bspline_optimizers_[1]->setPosAndAcc(pos_knots, acc_knots);
  }

  bspline_optimizers_[1]->setBoundaryStates(start, end, start_idx, end_idx);
  bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);
  bspline_optimizers_[1]->optimize(yaw, dt_yaw, cost_func, 3, 3);

  // Update traj info
  local_data_.yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
  local_data_.yawdot_traj_ = local_data_.yaw_traj_.getDerivative();
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();
  plan_data_.dt_yaw_ = dt_yaw;

  vector<Eigen::Vector3d> knot_points;
  local_data_.yaw_traj_.getKnotPoint(knot_points);
}

// A simple yaw planner without optimization that uniformly rotates yaw
void FastPlannerManager::planYawPreset(const Eigen::Vector3d &start_yaw, const double &end_yaw) {
  const int seg_num = 12;
  const double dt_yaw = local_data_.duration_ / seg_num; // time of B-spline segment

  // Start and end yaw
  double start_yaw2 = start_yaw[0];
  while (start_yaw2 < -M_PI)
    start_yaw2 += 2 * M_PI;
  while (start_yaw2 > M_PI)
    start_yaw2 -= 2 * M_PI;
  double last_yaw = start_yaw2;

  double end_yaw2 = end_yaw;
  calcNextYaw(last_yaw, end_yaw2);

  // Define waypoints such that yaw change is uniform
  double yaw_diff = end_yaw2 - start_yaw2;
  vector<Eigen::Vector3d> waypoints;
  for (int i = 0; i < seg_num + 1; i++) {
    double t = i * dt_yaw;
    double yaw = start_yaw2 + yaw_diff * t / local_data_.duration_;
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
  for (double ts = 0; ts <= 1e-3 + local_data_.duration_; ts += dt_yaw)
    point_set.push_back(poly.evaluate(ts, 0));

  boundary_der.push_back(poly.evaluate(0, 1));
  boundary_der.push_back(poly.evaluate(local_data_.duration_, 1));
  boundary_der.push_back(poly.evaluate(0, 2));
  boundary_der.push_back(poly.evaluate(local_data_.duration_, 2));

  Eigen::MatrixXd yaw;
  NonUniformBspline::parameterizeToBspline(dt_yaw, point_set, boundary_der, 3, yaw);

  // Update traj info
  local_data_.yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
  local_data_.yawdot_traj_ = local_data_.yaw_traj_.getDerivative();
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();
  plan_data_.dt_yaw_ = dt_yaw;
}

void FastPlannerManager::calcNextYaw(const double &last_yaw, double &yaw) {
  // round yaw to [-PI, PI]
  double round_last = last_yaw;
  while (round_last < -M_PI) {
    round_last += 2 * M_PI;
  }
  while (round_last > M_PI) {
    round_last -= 2 * M_PI;
  }

  double diff = yaw - round_last;
  if (fabs(diff) <= M_PI) {
    yaw = last_yaw + diff;
  } else if (diff > M_PI) {
    yaw = last_yaw + diff - 2 * M_PI;
  } else if (diff < -M_PI) {
    yaw = last_yaw + diff + 2 * M_PI;
  }
}

} // namespace fast_planner
