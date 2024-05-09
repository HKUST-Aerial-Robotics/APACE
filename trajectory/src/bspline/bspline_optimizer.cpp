#include "bspline/bspline_optimizer.h"

namespace fast_planner {
const int BsplineOptimizer::SMOOTHNESS = (1 << 0);
const int BsplineOptimizer::DISTANCE = (1 << 1);
const int BsplineOptimizer::FEASIBILITY = (1 << 2);
const int BsplineOptimizer::START = (1 << 3);
const int BsplineOptimizer::END = (1 << 4);
const int BsplineOptimizer::GUIDE = (1 << 5);
const int BsplineOptimizer::WAYPOINTS = (1 << 6);
const int BsplineOptimizer::VIEWCONS = (1 << 7);
const int BsplineOptimizer::MINTIME = (1 << 8);
const int BsplineOptimizer::YAWFEASIBILITY = (1 << 10);
const int BsplineOptimizer::PARALLAX = (1 << 11);
const int BsplineOptimizer::VERTICALVISIBILITY = (1 << 12);
const int BsplineOptimizer::YAWCOVISIBILITY = (1 << 13);

const int BsplineOptimizer::GUIDE_PHASE = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::GUIDE |
                                          BsplineOptimizer::START | BsplineOptimizer::END;
const int BsplineOptimizer::NORMAL_PHASE =
    BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::DISTANCE | BsplineOptimizer::FEASIBILITY |
    BsplineOptimizer::START | BsplineOptimizer::END;

void BsplineOptimizer::setParam(ros::NodeHandle &nh) {
  nh.param("optimization/visualization", vis_, false);

  nh.param("optimization/ld_smooth", ld_smooth_, -1.0);
  nh.param("optimization/ld_dist", ld_dist_, -1.0);
  nh.param("optimization/ld_feasi", ld_feasi_, -1.0);
  nh.param("optimization/ld_start", ld_start_, -1.0);
  nh.param("optimization/ld_end", ld_end_, -1.0);
  nh.param("optimization/ld_guide", ld_guide_, -1.0);
  nh.param("optimization/ld_waypt", ld_waypt_, -1.0);
  nh.param("optimization/ld_view", ld_view_, -1.0);
  nh.param("optimization/ld_time", ld_time_, -1.0);
  nh.param("optimization/ld_yaw_feasi", ld_yaw_feasi_, -1.0);
  nh.param("optimization/ld_parallax", ld_parallax_, -1.0);
  nh.param("optimization/ld_vertical_visibility", ld_vertical_visibility_, -1.0);
  nh.param("optimization/ld_yaw_covisibility", ld_yaw_covisib_, -1.0);

  nh.param("optimization/dist0", dist0_, -1.0);
  nh.param("optimization/max_vel", max_vel_, -1.0);
  nh.param("optimization/max_acc", max_acc_, -1.0);
  nh.param("optimization/max_vel_yaw", max_vel_yaw_, -1.0);
  nh.param("optimization/dlmin", dlmin_, -1.0);
  nh.param("optimization/wnl", wnl_, -1.0);

  nh.param("optimization/max_iteration_num1", max_iteration_num_[0], -1);
  nh.param("optimization/max_iteration_num2", max_iteration_num_[1], -1);
  nh.param("optimization/max_iteration_num3", max_iteration_num_[2], -1);
  nh.param("optimization/max_iteration_num4", max_iteration_num_[3], -1);
  nh.param("optimization/max_iteration_time1", max_iteration_time_[0], -1.0);
  nh.param("optimization/max_iteration_time2", max_iteration_time_[1], -1.0);
  nh.param("optimization/max_iteration_time3", max_iteration_time_[2], -1.0);
  nh.param("optimization/max_iteration_time4", max_iteration_time_[3], -1.0);

  nh.param("optimization/algorithm1", algorithm1_, -1);
  nh.param("optimization/algorithm2", algorithm2_, -1);
  nh.param("manager/bspline_degree", bspline_degree_, 3);

  time_lb_ = -1; // Not used by in most case
  vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("optimization/debug_vis", 10);
}

void BsplineOptimizer::setEnvironment(const EDTEnvironment::Ptr &env) {
  this->edt_environment_ = env;
}

void BsplineOptimizer::initParallaxUtil(ros::NodeHandle &nh) {
  ParallaxConfig config;
  nh.param("optimization/parallax/estimator_freq", config.estimator_freq_, -1.0);
  nh.param("optimization/parallax/max_parallax", config.max_parallax_, -1.0);
  nh.param("optimization/parallax/pot_a", config.pot_a_, -1.0);
  parallax_util_.reset(new ParallaxUtil(config));
}

void BsplineOptimizer::setCostFunction(const int &cost_code) {
  cost_function_ = cost_code;

  // Print cost function
  bool verbose = false;
  if (verbose) {
    string cost_str;
    if (cost_function_ & SMOOTHNESS)
      cost_str += "smooth | ";
    if (cost_function_ & DISTANCE)
      cost_str += " dist | ";
    if (cost_function_ & FEASIBILITY)
      cost_str += " feasi | ";
    if (cost_function_ & START)
      cost_str += " start | ";
    if (cost_function_ & END)
      cost_str += " end | ";
    if (cost_function_ & GUIDE)
      cost_str += " guide | ";
    if (cost_function_ & WAYPOINTS)
      cost_str += " waypt | ";
    if (cost_function_ & VIEWCONS)
      cost_str += " view | ";
    if (cost_function_ & MINTIME)
      cost_str += " time | ";
    if (cost_function_ & YAWFEASIBILITY)
      cost_str += " yaw_feasi | ";
    if (cost_function_ & PARALLAX)
      cost_str += " parallax | ";
    if (cost_function_ & VERTICALVISIBILITY)
      cost_str += " veritcal_visibility | ";
    if (cost_function_ & YAWCOVISIBILITY)
      cost_str += " yaw_covisibility | ";
    ROS_INFO_STREAM("cost func: " << cost_str);
  }
}

void BsplineOptimizer::setGuidePath(const vector<Eigen::Vector3d> &guide_pt) {
  guide_pts_ = guide_pt;
}

void BsplineOptimizer::setWaypoints(const vector<Eigen::Vector3d> &waypts,
                                    const vector<int> &waypt_idx) {
  waypoints_ = waypts;
  waypt_idx_ = waypt_idx;
}

void BsplineOptimizer::setPosAndAcc(const vector<Eigen::Vector3d> &pos,
                                    const vector<Eigen::Vector3d> &acc, const vector<int> &idx) {
  pos_ = pos;
  acc_ = acc;
  pos_idx_ = idx;
}
void BsplineOptimizer::setViewConstraint(const ViewConstraint &vc) { view_cons_ = vc; }

void BsplineOptimizer::setParallaxUtil(const ParallaxUtilPtr &pu) { parallax_util_ = pu; }

void BsplineOptimizer::setBoundaryStates(const vector<Eigen::Vector3d> &start,
                                         const vector<Eigen::Vector3d> &end,
                                         const vector<bool> &start_idx,
                                         const vector<bool> &end_idx) {
  start_state_ = start;
  end_state_ = end;
  start_con_index_ = start_idx;
  end_con_index_ = end_idx;
}

void BsplineOptimizer::setTimeLowerBound(const double &lb) { time_lb_ = lb; }

void BsplineOptimizer::optimize(Eigen::MatrixXd &points, double &dt, const int &cost_function,
                                const int &max_num_id, const int &max_time_id) {
  if (start_state_.empty()) {
    ROS_ERROR("Initial state undefined!");
    return;
  }
  control_points_ = points;
  knot_span_ = dt;
  max_num_id_ = max_num_id;
  max_time_id_ = max_time_id;
  setCostFunction(cost_function);

  // Set necessary data and flag
  dim_ = control_points_.cols();
  if (dim_ == 1)
    order_ = 3;
  else
    order_ = bspline_degree_;
  point_num_ = control_points_.rows();
  optimize_time_ = cost_function_ & MINTIME;
  variable_num_ = optimize_time_ ? dim_ * point_num_ + 1 : dim_ * point_num_;
  if (variable_num_ <= 0) {
    ROS_ERROR("Empty varibale to optimization solver.");
    return;
  }

  // ROS_ERROR("point_num_: %d, variable_num_: %d", point_num_, variable_num_);
  // ROS_ERROR("variable_num_: %d", variable_num_);

  pt_dist_ = 0.0;
  for (int i = 0; i < control_points_.rows() - 1; ++i) {
    pt_dist_ += (control_points_.row(i + 1) - control_points_.row(i)).norm();
  }
  pt_dist_ /= double(point_num_);

  iter_num_ = 0;
  min_cost_ = std::numeric_limits<double>::max();
  g_q_.resize(point_num_);
  g_smoothness_.resize(point_num_);
  g_distance_.resize(point_num_);
  g_feasibility_.resize(point_num_);
  g_start_.resize(point_num_);
  g_end_.resize(point_num_);
  g_guide_.resize(point_num_);
  g_waypoints_.resize(point_num_);
  g_view_.resize(point_num_);
  g_time_.resize(point_num_);
  g_yaw_feasibility_.resize(point_num_);
  g_parallax_.resize(point_num_);
  g_vertical_visib_.resize(point_num_);
  g_yaw_covisib_.resize(point_num_);

  comb_time = 0.0;

  optimize();

  points = control_points_;
  dt = knot_span_;
  start_state_.clear();
  start_con_index_.clear();
  end_con_index_.clear();
  time_lb_ = -1;
}

void BsplineOptimizer::optimize() {
  // Optimize all control points and maybe knot span dt
  // Use NLopt solver
  nlopt::opt opt(nlopt::algorithm(isQuadratic() ? algorithm1_ : algorithm2_), variable_num_);
  opt.set_min_objective(BsplineOptimizer::costFunction, this);
  opt.set_xtol_rel(1e-4);

  // if (!vis_) {
  //   opt.set_maxeval(max_iteration_num_[max_num_id_]);
  //   ROS_WARN("max iteration num: %d", max_iteration_num_[max_num_id_]);
  //   opt.set_maxtime(max_iteration_time_[max_time_id_]);
  //   ROS_WARN("max iteration time: %f", max_iteration_time_[max_time_id_]);
  // }

  // Set axis aligned bounding box for optimization
  Eigen::Vector3d bmin, bmax;
  edt_environment_->map_server_->getBox(bmin, bmax);
  for (int k = 0; k < 3; ++k) {
    bmin[k] += 0.1;
    bmax[k] -= 0.1;
  }

  vector<double> q(variable_num_);
  // Variables for control points
  for (int i = 0; i < point_num_; ++i)
    for (int j = 0; j < dim_; ++j) {
      double cij = control_points_(i, j);
      if (dim_ != 1)
        cij = max(min(cij, bmax[j % 3]), bmin[j % 3]);
      q[dim_ * i + j] = cij;
    }
  // Variables for knot span
  if (optimize_time_)
    q[variable_num_ - 1] = knot_span_;

  if (dim_ != 1) {
    vector<double> lb(variable_num_), ub(variable_num_);
    const double bound = 10.0;
    for (int i = 0; i < 3 * point_num_; ++i) {
      lb[i] = q[i] - bound;
      ub[i] = q[i] + bound;
      lb[i] = max(lb[i], bmin[i % 3]);
      ub[i] = min(ub[i], bmax[i % 3]);
    }
    if (optimize_time_) {
      lb[variable_num_ - 1] = 0.0;
      ub[variable_num_ - 1] = 5.0;
    }
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
  }

  // ROS_ERROR("q size: %d", q.size());

  auto t1 = ros::Time::now();
  try {
    double final_cost;
    nlopt::result result = opt.optimize(q, final_cost);
  } catch (std::exception &e) {
    cout << e.what() << endl;
  }

  // ROS_ERROR("best_variable_ size: %d", best_variable_.size());

  for (int i = 0; i < point_num_; ++i)
    for (int j = 0; j < dim_; ++j)
      control_points_(i, j) = best_variable_[dim_ * i + j];
  if (optimize_time_)
    knot_span_ = best_variable_[variable_num_ - 1];
}

void BsplineOptimizer::calcSmoothnessCost(const vector<Eigen::Vector3d> &q, const double &dt,
                                          double &cost, vector<Eigen::Vector3d> &gradient_q,
                                          double &gt) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);
  Eigen::Vector3d jerk, temp_j;

  for (int i = 0; i < q.size() - 3; i++) {
    /* evaluate jerk */
    // 3-rd order derivative = 1/(ts)^3*(q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1] - q[i])
    Eigen::Vector3d ji = (q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1] - q[i]) / pt_dist_;
    cost += ji.squaredNorm();
    temp_j = 2 * ji / pt_dist_;

    /* jerk gradient_q */
    // d cost / d q[i] = d cost / d jerk * d jerk / d q[i]
    // gradient_q = d cost / d q[i] for each i
    gradient_q[i + 0] += -temp_j;
    gradient_q[i + 1] += 3.0 * temp_j;
    gradient_q[i + 2] += -3.0 * temp_j;
    gradient_q[i + 3] += temp_j;
    // if (optimize_time_)
    //   gt += -6 * ji.dot(ji) / dt;
  }
}

void BsplineOptimizer::calcDistanceCost(const vector<Eigen::Vector3d> &q, double &cost,
                                        vector<Eigen::Vector3d> &gradient_q) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);

  double dist;
  Eigen::Vector3d dist_grad, g_zero(0, 0, 0);
  for (int i = 0; i < q.size(); i++) {
    edt_environment_->evaluateEDTWithGrad(q[i], -1.0, dist, dist_grad);
    if (dist_grad.norm() > 1e-4)
      dist_grad.normalize();

    if (dist < dist0_) {
      cost += pow(dist - dist0_, 2);
      gradient_q[i] += 2.0 * (dist - dist0_) * dist_grad;
    }
  }
}

void BsplineOptimizer::calcFeasibilityCost(const vector<Eigen::Vector3d> &q, const double &dt,
                                           double &cost, vector<Eigen::Vector3d> &gradient_q,
                                           double &gt) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);
  gt = 0.0;

  // Abbreviation of params
  const double dt_inv = 1 / dt;
  const double dt_inv2 = dt_inv * dt_inv;

  for (int i = 0; i < q.size() - 1; ++i) {
    // Control point of velocity
    Eigen::Vector3d vi = (q[i + 1] - q[i]) * dt_inv;
    for (int k = 0; k < 3; ++k) {
      // Calculate cost for each axis
      double vd = fabs(vi[k]) - max_vel_;
      if (vd > 0.0) {
        cost += pow(vd, 2);
        double sign = vi[k] > 0 ? 1.0 : -1.0;
        double tmp = 2 * vd * sign * dt_inv;
        gradient_q[i][k] += -tmp;
        gradient_q[i + 1][k] += tmp;
        if (optimize_time_)
          gt += tmp * (-vi[k]);
      }
    }
  }

  // Acc feasibility cost
  for (int i = 0; i < q.size() - 2; ++i) {
    Eigen::Vector3d ai = (q[i + 2] - 2 * q[i + 1] + q[i]) * dt_inv2;
    for (int k = 0; k < 3; ++k) {
      double ad = fabs(ai[k]) - max_acc_;
      if (ad > 0.0) {
        cost += pow(ad, 2);
        double sign = ai[k] > 0 ? 1.0 : -1.0;
        double tmp = 2 * ad * sign * dt_inv2;
        gradient_q[i][k] += tmp;
        gradient_q[i + 1][k] += -2 * tmp;
        gradient_q[i + 2][k] += tmp;
        if (optimize_time_)
          gt += tmp * ai[k] * (-2) * dt;
      }
    }
  }
}

void BsplineOptimizer::calcStartCost(const vector<Eigen::Vector3d> &q, const double &dt,
                                     double &cost, vector<Eigen::Vector3d> &gradient_q,
                                     double &gt) {
  CHECK_EQ(start_con_index_.size(), 3) << "Start state constraint is not set!";

  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  // std::fill(gradient_q.begin(), gradient_q.end(), zero);
  for (int i = 0; i < 3; ++i)
    gradient_q[i] = zero;
  gt = 0.0;

  Eigen::Vector3d q1, q2, q3, dq;
  q1 = q[0];
  q2 = q[1];
  q3 = q[2];

  // Start position
  if (start_con_index_[0]) {
    static const double w_pos = 10.0;
    dq = 1 / 6.0 * (q1 + 4 * q2 + q3) - start_state_[0];
    cost += w_pos * dq.squaredNorm();
    gradient_q[0] += w_pos * 2 * dq * (1 / 6.0);
    gradient_q[1] += w_pos * 2 * dq * (4 / 6.0);
    gradient_q[2] += w_pos * 2 * dq * (1 / 6.0);
  }

  // Start velocity
  if (start_con_index_[1]) {
    dq = 1 / (2 * dt) * (q3 - q1) - start_state_[1];
    cost += dq.squaredNorm();
    gradient_q[0] += 2 * dq * (-1.0) / (2 * dt);
    gradient_q[2] += 2 * dq * 1.0 / (2 * dt);
    if (optimize_time_)
      gt += dq.dot(q3 - q1) / (-dt * dt);
  }

  // Start acceleration
  if (start_con_index_[2]) {
    dq = 1 / (dt * dt) * (q1 - 2 * q2 + q3) - start_state_[2];
    cost += dq.squaredNorm();
    gradient_q[0] += 2 * dq * 1.0 / (dt * dt);
    gradient_q[1] += 2 * dq * (-2.0) / (dt * dt);
    gradient_q[2] += 2 * dq * 1.0 / (dt * dt);
    if (optimize_time_)
      gt += dq.dot(q1 - 2 * q2 + q3) / (-dt * dt * dt);
  }
}

void BsplineOptimizer::calcEndCost(const vector<Eigen::Vector3d> &q, const double &dt, double &cost,
                                   vector<Eigen::Vector3d> &gradient_q, double &gt) {
  CHECK_EQ(end_con_index_.size(), 3) << "End state constraint is not set!";

  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  // std::fill(gradient_q.begin(), gradient_q.end(), zero);
  for (int i = q.size() - 3; i < q.size(); ++i)
    gradient_q[i] = zero;
  gt = 0.0;

  Eigen::Vector3d q_3, q_2, q_1, dq;
  q_3 = q[q.size() - 3];
  q_2 = q[q.size() - 2];
  q_1 = q[q.size() - 1];

  // End position
  if (end_con_index_[0]) {
    dq = 1 / 6.0 * (q_1 + 4 * q_2 + q_3) - end_state_[0];
    cost += dq.squaredNorm();
    gradient_q[q.size() - 1] += 2 * dq * (1 / 6.0);
    gradient_q[q.size() - 2] += 2 * dq * (4 / 6.0);
    gradient_q[q.size() - 3] += 2 * dq * (1 / 6.0);
  }

  // End velocity
  if (end_con_index_[1]) {
    dq = 1 / (2 * dt) * (q_1 - q_3) - end_state_[1];
    cost += dq.squaredNorm();
    gradient_q[q.size() - 1] += 2 * dq * 1.0 / (2 * dt);
    gradient_q[q.size() - 3] += 2 * dq * (-1.0) / (2 * dt);
    if (optimize_time_)
      gt += dq.dot(q_1 - q_3) / (-dt * dt);
  }

  // End acceleration
  if (end_con_index_[2]) {
    dq = 1 / (dt * dt) * (q_1 - 2 * q_2 + q_3) - end_state_[2];
    cost += dq.squaredNorm();
    gradient_q[q.size() - 1] += 2 * dq * 1.0 / (dt * dt);
    gradient_q[q.size() - 2] += 2 * dq * (-2.0) / (dt * dt);
    gradient_q[q.size() - 3] += 2 * dq * 1.0 / (dt * dt);
    if (optimize_time_)
      gt += dq.dot(q_1 - 2 * q_2 + q_3) / (-dt * dt * dt);
  }
}

void BsplineOptimizer::calcWaypointsCost(const vector<Eigen::Vector3d> &q, double &cost,
                                         vector<Eigen::Vector3d> &gradient_q) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);

  Eigen::Vector3d q1, q2, q3, dq;

  // for (auto wp : waypoints_) {
  for (int i = 0; i < waypoints_.size(); ++i) {
    Eigen::Vector3d waypt = waypoints_[i];
    int idx = waypt_idx_[i];

    q1 = q[idx];
    q2 = q[idx + 1];
    q3 = q[idx + 2];

    dq = 1 / 6.0 * (q1 + 4 * q2 + q3) - waypt;
    cost += dq.squaredNorm();

    gradient_q[idx] += dq * (2.0 / 6.0);     // 2*dq*(1/6)
    gradient_q[idx + 1] += dq * (8.0 / 6.0); // 2*dq*(4/6)
    gradient_q[idx + 2] += dq * (2.0 / 6.0);
  }
}

/* use the uniformly sampled points on a geomertic path to guide the
 * trajectory. For each control points to be optimized, it is assigned a
 * guiding point on the path and the distance between them is penalized */
void BsplineOptimizer::calcGuideCost(const vector<Eigen::Vector3d> &q, double &cost,
                                     vector<Eigen::Vector3d> &gradient_q) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);

  int end_idx = q.size() - order_;

  for (int i = order_; i < end_idx; i++) {
    Eigen::Vector3d gpt = guide_pts_[i - order_];
    cost += (q[i] - gpt).squaredNorm();
    gradient_q[i] += 2 * (q[i] - gpt);
  }
}

void BsplineOptimizer::calcViewCost(const vector<Eigen::Vector3d> &q, double &cost,
                                    vector<Eigen::Vector3d> &gradient_q) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);
  Eigen::Vector3d p = view_cons_.pt_;
  Eigen::Vector3d v = view_cons_.dir_.normalized();
  Eigen::Matrix3d vvT = v * v.transpose();
  Eigen::Matrix3d I_vvT = Eigen::Matrix3d::Identity() - vvT;

  // prependicular cost, increase visibility of points before blocked point
  int i = view_cons_.idx_;
  Eigen::Vector3d dn = (q[i] - p) - ((q[i] - p).dot(v)) * v;
  cost += dn.squaredNorm();
  gradient_q[i] += 2 * I_vvT * dn;
  double norm_dn = dn.norm();

  // parallel cost, increase projection along view direction
  Eigen::Vector3d dl = ((q[i] - p).dot(v)) * v;
  double norm_dl = dl.norm();
  double safe_dist = view_cons_.dir_.norm();
  if (norm_dl < safe_dist) {
    cost += wnl_ * pow(norm_dl - safe_dist, 2);
    gradient_q[i] += wnl_ * 2 * (norm_dl - safe_dist) * vvT * dl / norm_dl;
  }
}

void BsplineOptimizer::calcTimeCost(const double &dt, double &cost, double &gt) {
  // Min time
  double duration = (point_num_ - order_) * dt;
  cost = duration;
  // cost = pow(duration, 2);
  gt = double(point_num_ - order_);

  // Time lower bound
  bool penalized = false;
  double p_value = 0.0;
  if (time_lb_ > 0 && duration < time_lb_) {
    // ROS_ERROR("***************time_lb_ is penalized!");
    penalized = true;
    // static const double w_lb = 10;
    static const double w_lb = 10000;
    p_value = w_lb * pow(duration - time_lb_, 2);
    cost += w_lb * pow(duration - time_lb_, 2);
    gt += w_lb * 2 * (duration - time_lb_) * (point_num_ - order_);
  }

  // ROS_INFO("Time cost is %f", duration);
  // if(penalized)
  //   ROS_INFO("Time lower bound is penalized by %f", p_value);
  // else
  //   ROS_INFO("Time lower bound is not penalized, time lower bound is %f", time_lb_);
}

void BsplineOptimizer::calcParallaxCost(const vector<Eigen::Vector3d> &q, const double &dt,
                                        double &cost, vector<Eigen::Vector3d> &gradient_q) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);

  double pot_cost;
  vector<Eigen::Vector3d> dpot_dq;
  vector<double> part1, part2;
  for (int i = 0; i < q.size() - 3; ++i) {
    // For (q0, q1, q2, q3)->(knot1, knot2), calculate the parallax cost and gradient
    // TicToc tic1;
    vector<Eigen::Vector3d> q_cur;
    for (int j = 0; j < 4; j++) {
      q_cur.push_back(q[i + j]);
    }

    Eigen::Vector3d knot_mid =
        1 / 12.0 * ((q_cur[0] + 4 * q_cur[1] + q_cur[2]) + (q_cur[1] + 4 * q_cur[2] + q_cur[3]));
    vector<Eigen::Vector3d> features;
    edt_environment_->getFeaturesInFovDepth(knot_mid, features);

    // part1.push_back(tic1.toc() * 1000);

    parallax_util_->calcParaCostAndGradientsKnots(q_cur, dt, features, pot_cost, dpot_dq);
    // parallax_util_->calcParaCostAndGradientsKnots(q_cur, knot_nn_features_[i], pot_cost,
    // dpot_dq);

    cost += pot_cost;
    for (int j = 0; j < 4; j++) {
      gradient_q[i + j] += dpot_dq[j];
    }
    // part2.push_back(tic2.toc() * 1000);
  }

  // sum up part1 and part2
  // double sum1 = std::accumulate(part1.begin(), part1.end(), 0.0);
  // double sum2 = std::accumulate(part2.begin(), part2.end(), 0.0);
  // ROS_INFO("part1: %f, part2: %f", sum1, sum2);
}

void BsplineOptimizer::calcVerticalCoVisbilityCost(const vector<Eigen::Vector3d> &q,
                                                   const double &dt, double &cost,
                                                   vector<Eigen::Vector3d> &gradient_q) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);

  double pot_cost;
  vector<Eigen::Vector3d> dpot_dq;
  vector<double> part1, part2;
  for (int i = 0; i < q.size() - 3; ++i) {
    // For (q0, q1, q2, q3)->(knot1, knot2), calculate the covisibility cost and gradient
    // TicToc tic1;
    vector<Eigen::Vector3d> q_cur;
    for (int j = 0; j < 4; j++) {
      q_cur.push_back(q[i + j]);
    }

    Eigen::Vector3d knot_mid =
        1 / 12.0 * ((q_cur[0] + 4 * q_cur[1] + q_cur[2]) + (q_cur[1] + 4 * q_cur[2] + q_cur[3]));
    vector<Eigen::Vector3d> features;
    edt_environment_->getFeaturesInFovDepth(knot_mid, features);

    // part1.push_back(tic1.toc() * 1000);

    // parallax_util_->calcVCVCostAndGradientsKnots(q_cur, knot_nn_features_[i], dt, pot_cost,
    // dpot_dq);
    parallax_util_->calcVCVCostAndGradientsKnots(q_cur, features, dt, pot_cost, dpot_dq);

    cost += pot_cost;
    for (int j = 0; j < 4; j++) {
      gradient_q[i + j] += dpot_dq[j];
    }
    // part2.push_back(tic2.toc() * 1000);
  }

  // sum up part1 and part2
  // double sum1 = std::accumulate(part1.begin(), part1.end(), 0.0);
  // double sum2 = std::accumulate(part2.begin(), part2.end(), 0.0);
  // ROS_INFO("part1: %f, part2: %f", sum1, sum2);
}

void BsplineOptimizer::calcPerceptionCost(const vector<Eigen::Vector3d> &q, const double &dt,
                                          double &cost, vector<Eigen::Vector3d> &gradient_q,
                                          const double ld_para, const double ld_vcv) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);

  double pot_cost_para, pot_cost_vcv;
  vector<Eigen::Vector3d> dpot_dq_para, dpot_dq_vcv;
  for (int i = 0; i < q.size() - 3; ++i) {
    // For (q0, q1, q2, q3)->(knot1, knot2), calculate the parallax cost and gradient
    // TicToc tic1;
    vector<Eigen::Vector3d> q_cur;
    for (int j = 0; j < 4; j++) {
      q_cur.push_back(q[i + j]);
    }

    Eigen::Vector3d knot_mid =
        1 / 12.0 * ((q_cur[0] + 4 * q_cur[1] + q_cur[2]) + (q_cur[1] + 4 * q_cur[2] + q_cur[3]));
    vector<Eigen::Vector3d> features;
    edt_environment_->getFeaturesInFovDepth(knot_mid, features);

    parallax_util_->calcParaCostAndGradientsKnots(q_cur, dt, features, pot_cost_para, dpot_dq_para);
    parallax_util_->calcVCVCostAndGradientsKnots(q_cur, features, dt, pot_cost_vcv, dpot_dq_vcv);

    cost += ld_para * pot_cost_para;
    cost += ld_vcv * pot_cost_vcv;
    for (int j = 0; j < 4; j++) {
      gradient_q[i + j] += ld_para * dpot_dq_para[j];
      gradient_q[i + j] += ld_vcv * dpot_dq_vcv[j];
    }
  }
}

void BsplineOptimizer::calcYawCoVisbilityCost(const vector<Eigen::Vector3d> &q, const double &dt,
                                              double &cost, vector<Eigen::Vector3d> &gradient_q) {
  // q.size = n+1, pos_.size = n-p+2 = (n+1) - 2, where p = 3
  CHECK_EQ(q.size() - 2, pos_.size()) << "q and pos_ have incompatible size!";

  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);

  double pot_cost;
  vector<Eigen::Vector3d> dpot_dq;
  for (int i = 0; i < q.size() - 3; ++i) {
    // For (q0, q1, q2, q3)->(knot1, knot2), calculate the covisibility cost and gradient
    vector<Eigen::Vector3d> q_cur;
    for (int j = 0; j < 4; j++) {
      q_cur.push_back(q[i + j]);
    }

    vector<Eigen::Vector3d> knots_pos, knots_acc;
    for (int j = 0; j < 2; j++) {
      knots_pos.push_back(pos_[i + j]);
      knots_acc.push_back(acc_[i + j]);
    }

    Eigen::Vector3d knot_mid = (pos_[i] + pos_[i + 1]) / 2.0;
    vector<Eigen::Vector3d> features;
    edt_environment_->getFeaturesInFovDepth(knot_mid, features);

    parallax_util_->calcYawCVCostAndGradientsKnots(q_cur, knots_pos, knots_acc, features, pot_cost,
                                                   dpot_dq);

    cost += pot_cost;
    for (int j = 0; j < 4; j++) {
      gradient_q[i + j] += dpot_dq[j];
    }
  }
}

void BsplineOptimizer::calcVerticalVisbilityCost(const vector<Eigen::Vector3d> &q, const double &dt,
                                                 double &cost,
                                                 vector<Eigen::Vector3d> &gradient_q) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);

  double pot_cost;
  vector<Eigen::Vector3d> dpot_dq;
  vector<double> part1, part2;
  for (int i = 0; i < q.size() - 2; ++i) {
    // For (q0, q1, q2)-> knot, calculate the vertical visbility cost and gradient
    TicToc tic1;
    vector<Eigen::Vector3d> q_cur;
    for (int j = 0; j < 3; j++) {
      q_cur.push_back(q[i + j]);
    }
    Eigen::Vector3d knot = 1 / 6.0 * (q_cur[0] + 4 * q_cur[1] + q_cur[2]);

    vector<Eigen::Vector3d> features;
    edt_environment_->getFeaturesInFovDepth(knot, features);

    part1.push_back(tic1.toc() * 1000);

    TicToc tic2;
    parallax_util_->calcVVCostAndGradientsKnots(q_cur, features, dt, pot_cost, dpot_dq);

    cost += pot_cost;
    for (int j = 0; j < 3; j++) {
      gradient_q[i + j] += dpot_dq[j];
    }
    part2.push_back(tic2.toc() * 1000);
  }

  // sum up part1 and part2
  double sum1 = std::accumulate(part1.begin(), part1.end(), 0.0);
  double sum2 = std::accumulate(part2.begin(), part2.end(), 0.0);
  // ROS_INFO("part1: %f, part2: %f", sum1, sum2);
}

void BsplineOptimizer::calcYawFeasibilityCost(const vector<Eigen::Vector3d> &q, const double &dt,
                                              double &cost, vector<Eigen::Vector3d> &gradient_q) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);

  // Abbreviation of params
  const double dt_inv = 1 / dt;
  const double dt_inv2 = dt_inv * dt_inv;

  for (int i = 0; i < q.size() - 1; ++i) {
    // Control point of velocity
    Eigen::Vector3d vi = (q[i + 1] - q[i]) * dt_inv;
    double vd = fabs(vi[0]) - max_vel_yaw_;
    if (vd > 0.0) {
      cost += pow(vd, 2);
      double sign = vi[0] > 0 ? 1.0 : -1.0;
      double tmp = 2 * vd * sign * dt_inv;
      gradient_q[i][0] += -tmp;
      gradient_q[i + 1][0] += tmp;
      // std::cout << "vi: " << vi[0] << std::endl;
      // std::cout << "vd: " << vd << std::endl;
      // std::cout << "cost +=  " << pow(vd, 2) << std::endl;
    }
  }
}

void BsplineOptimizer::calcNNFeaturesAtKnotPoints(const vector<Eigen::Vector3d> &q) {
  for (int i = 0; i < q.size() - 3; ++i) {
    // For midpoint of (q0, q1, q2)-> knot1, (q1, q2, q3)-> knot2, calculate the neighboring
    // features index
    Eigen::Vector3d knot_mid =
        1 / 12.0 * ((q[i] + 4 * q[i + 1] + q[i + 2]) + (q[i + 1] + 4 * q[i + 2] + q[i + 3]));

    vector<Eigen::Vector3d> features;
    edt_environment_->getFeaturesInFovDepth(knot_mid, features);

    knot_nn_features_.push_back(features);
  }
}

void BsplineOptimizer::combineCost(const std::vector<double> &x, std::vector<double> &grad,
                                   double &f_combine) {
  // This solver can support 1D-3D B-spline optimization, but we use Vector3d to store each control
  // point. For 1D case, the second and third elements are zero, and similar for the 2D case.

  ros::Time t1 = ros::Time::now();

  for (int i = 0; i < point_num_; ++i) {
    for (int j = 0; j < dim_; ++j)
      g_q_[i][j] = x[dim_ * i + j];
    for (int j = dim_; j < 3; ++j)
      g_q_[i][j] = 0.0;
  }
  const double dt = optimize_time_ ? x[variable_num_ - 1] : knot_span_;

  f_combine = 0.0;
  grad.resize(variable_num_);
  fill(grad.begin(), grad.end(), 0.0);

  bool verbose = false;
  double f_smoothness, gt_smoothness, f_distance, f_feasibility, gt_feasibility, f_start, gt_start,
      f_end, gt_end, f_guide, f_waypoints, f_view, f_time, gt_time, f_info, f_yaw_feasibility,
      f_parallax, f_vertical_visbility, f_yaw_covisibility = 0.0;

  // if (cost_function_ & PARALLAX || cost_function_ & VERTICALVISIBILITY) {
  //   calcNNFeaturesAtKnotPoints(g_q_);
  // }
  if (cost_function_ & SMOOTHNESS) {
    calcSmoothnessCost(g_q_, dt, f_smoothness, g_smoothness_, gt_smoothness);
    f_combine += ld_smooth_ * f_smoothness;
    for (int i = 0; i < point_num_; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += ld_smooth_ * g_smoothness_[i](j);
    if (optimize_time_)
      grad[variable_num_ - 1] += ld_smooth_ * gt_smoothness;
  }
  if (cost_function_ & DISTANCE) {
    calcDistanceCost(g_q_, f_distance, g_distance_);
    f_combine += ld_dist_ * f_distance;
    for (int i = 0; i < point_num_; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += ld_dist_ * g_distance_[i](j);
  }
  if (cost_function_ & FEASIBILITY) {
    calcFeasibilityCost(g_q_, dt, f_feasibility, g_feasibility_, gt_feasibility);
    f_combine += ld_feasi_ * f_feasibility;
    for (int i = 0; i < point_num_; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += ld_feasi_ * g_feasibility_[i](j);
    if (optimize_time_)
      grad[variable_num_ - 1] += ld_feasi_ * gt_feasibility;
  }
  if (cost_function_ & START) {
    calcStartCost(g_q_, dt, f_start, g_start_, gt_start);
    f_combine += ld_start_ * f_start;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += ld_start_ * g_start_[i](j);
    if (optimize_time_)
      grad[variable_num_ - 1] += ld_start_ * gt_start;
  }
  if (cost_function_ & END) {
    calcEndCost(g_q_, dt, f_end, g_end_, gt_end);
    f_combine += ld_end_ * f_end;
    for (int i = point_num_ - 3; i < point_num_; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += ld_end_ * g_end_[i](j);
    if (optimize_time_)
      grad[variable_num_ - 1] += ld_end_ * gt_end;
  }
  if (cost_function_ & GUIDE) {
    calcGuideCost(g_q_, f_guide, g_guide_);
    f_combine += ld_guide_ * f_guide;
    for (int i = 0; i < point_num_; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += ld_guide_ * g_guide_[i](j);
  }
  if (cost_function_ & WAYPOINTS) {
    calcWaypointsCost(g_q_, f_waypoints, g_waypoints_);
    f_combine += ld_waypt_ * f_waypoints;
    for (int i = 0; i < point_num_; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += ld_waypt_ * g_waypoints_[i](j);
  }
  if (cost_function_ & VIEWCONS) {
    calcViewCost(g_q_, f_view, g_view_);
    f_combine += ld_view_ * f_view;
    for (int i = 0; i < point_num_; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += ld_view_ * g_view_[i](j);
  }
  if (cost_function_ & MINTIME) {
    calcTimeCost(dt, f_time, gt_time);
    f_combine += ld_time_ * f_time;
    grad[variable_num_ - 1] += ld_time_ * gt_time;
  }
  if (cost_function_ & YAWFEASIBILITY) {
    calcYawFeasibilityCost(g_q_, dt, f_yaw_feasibility, g_yaw_feasibility_);
    f_combine += ld_yaw_feasi_ * f_yaw_feasibility;
    for (int i = 0; i < point_num_; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += ld_yaw_feasi_ * g_yaw_feasibility_[i](j);
  }
  // if (cost_function_ & PARALLAX) {
  //   TicToc tic;
  //   calcParallaxCost(g_q_, dt, f_parallax, g_parallax_);
  //   f_combine += ld_parallax_ * f_parallax;
  //   for (int i = 0; i < point_num_; i++)
  //     for (int j = 0; j < dim_; j++)
  //       grad[dim_ * i + j] += ld_parallax_ * g_parallax_[i](j);
  //   if (verbose)
  //     ROS_INFO("PARALLAX time: %f ms", tic.toc() * 1000);
  // }
  // if (cost_function_ & VERTICALVISIBILITY) {
  //   calcVerticalCoVisbilityCost(g_q_, dt, f_vertical_visbility, g_vertical_visib_);
  //   f_combine += ld_vertical_visibility_ * f_vertical_visbility;
  //   for (int i = 0; i < point_num_; i++)
  //     for (int j = 0; j < dim_; j++)
  //       grad[dim_ * i + j] += ld_vertical_visibility_ * g_vertical_visib_[i](j);
  // }
  if ((cost_function_ & PARALLAX) && (cost_function_ & VERTICALVISIBILITY)) {
    calcPerceptionCost(g_q_, dt, f_parallax, g_parallax_, ld_parallax_, ld_vertical_visibility_);
    f_combine += f_parallax;
    for (int i = 0; i < point_num_; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += g_parallax_[i](j);
  }
  if (cost_function_ & YAWCOVISIBILITY) {
    calcYawCoVisbilityCost(g_q_, dt, f_yaw_covisibility, g_yaw_covisib_);
    f_combine += ld_yaw_covisib_ * f_yaw_covisibility;
    for (int i = 0; i < point_num_; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += ld_yaw_covisib_ * g_yaw_covisib_[i](j);
  }

  verbose = false;

  if (verbose) {
    if (cost_function_ & SMOOTHNESS)
      ROS_INFO("Smoothness cost:      %f", ld_smooth_ * f_smoothness);
    if (cost_function_ & DISTANCE)
      ROS_INFO("Distance cost:        %f", ld_dist_ * f_distance);
    if (cost_function_ & FEASIBILITY)
      ROS_INFO("Feasibility cost:     %f", ld_feasi_ * f_feasibility);
    if (cost_function_ & START)
      ROS_INFO("Start cost:           %f", ld_start_ * f_start);
    if (cost_function_ & END)
      ROS_INFO("End cost:             %f", ld_end_ * f_end);
    if (cost_function_ & GUIDE)
      ROS_INFO("Guide cost:           %f", ld_guide_ * f_guide);
    if (cost_function_ & WAYPOINTS)
      ROS_INFO("Waypoint cost:        %f", ld_waypt_ * f_waypoints);
    if (cost_function_ & VIEWCONS)
      ROS_INFO("View cost:            %f", ld_view_ * f_view);
    if (cost_function_ & MINTIME)
      ROS_INFO("Time cost:            %f", ld_time_ * f_time);
    if (cost_function_ & YAWFEASIBILITY)
      ROS_INFO("Yaw Feasibility cost: %f", ld_yaw_feasi_ * f_yaw_feasibility);
    if (cost_function_ & PARALLAX)
      ROS_INFO("Parallax cost:        %f", ld_parallax_ * f_parallax);
    if (cost_function_ & VERTICALVISIBILITY)
      ROS_INFO("Vertical Visib cost:  %f", ld_vertical_visibility_ * f_vertical_visbility);
    if (cost_function_ & YAWCOVISIBILITY)
      ROS_INFO("Yaw Covisib cost:     %f", ld_yaw_covisib_ * f_yaw_covisibility);
    ROS_INFO("--------------------");
    ROS_INFO("TOTAL:                %f", f_combine);
    ROS_INFO("------------------------------");
  }
  comb_time += (ros::Time::now() - t1).toSec();

  // Visualize intermediate results
  if (vis_) {
    vector<Eigen::Vector3d> grad_3d;
    for (int i = 0; i < point_num_; i++) {
      Eigen::Vector3d temp;
      for (int j = 0; j < dim_; j++) {
        temp(j) = grad[dim_ * i + j];
      }
      grad_3d.push_back(temp);
    }
    debugVisualization(g_q_, grad_3d);
    ros::Duration(0.05).sleep();
  }
}

double BsplineOptimizer::costFunction(const std::vector<double> &x, std::vector<double> &grad,
                                      void *func_data) {
  BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);
  double cost;
  opt->combineCost(x, grad, cost);
  opt->iter_num_++;

  /* save the min cost result */
  if (cost < opt->min_cost_) {
    opt->min_cost_ = cost;
    opt->best_variable_ = x;
  }
  return cost;

  // /* evaluation */
  // ros::Time te1 = ros::Time::now();
  // double time_now = (te1 - opt->time_start_).toSec();
  // opt->vec_time_.push_back(time_now);
  // if (opt->vec_cost_.size() == 0)
  // {
  //   opt->vec_cost_.push_back(f_combine);
  // }
  // else if (opt->vec_cost_.back() > f_combine)
  // {
  //   opt->vec_cost_.push_back(f_combine);
  // }
  // else
  // {
  //   opt->vec_cost_.push_back(opt->vec_cost_.back());
  // }
}

vector<Eigen::Vector3d> BsplineOptimizer::matrixToVectors(const Eigen::MatrixXd &ctrl_pts) {
  vector<Eigen::Vector3d> ctrl_q;
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    ctrl_q.push_back(ctrl_pts.row(i));
  }
  return ctrl_q;
}

Eigen::MatrixXd BsplineOptimizer::getControlPoints() { return this->control_points_; }

bool BsplineOptimizer::isQuadratic() {
  if (cost_function_ == GUIDE_PHASE) {
    return true;
  } else if (cost_function_ == SMOOTHNESS) {
    return true;
  } else if (cost_function_ == (SMOOTHNESS | WAYPOINTS)) {
    return true;
  }
  return false;
}

void BsplineOptimizer::debugVisualization(const std::vector<Eigen::Vector3d> &q,
                                          const std::vector<Eigen::Vector3d> &q_grad) {
  if (vis_pub_.getNumSubscribers() == 0)
    return;

  visualization_msgs::MarkerArray marker_array;
  for (size_t i = 0; i < q.size(); ++i) {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = i;
    mk.type = visualization_msgs::Marker::ARROW;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.w = 1.0;
    mk.scale.x = 0.1;
    mk.scale.y = 0.2;
    mk.scale.z = 0.3;
    mk.color.r = 1.0;
    mk.color.g = 0.5;
    mk.color.b = 0.0;
    mk.color.a = 1.0;

    geometry_msgs::Point pt;
    pt.x = q[i](0);
    pt.y = q[i](1);
    pt.z = q[i](2);
    mk.points.push_back(pt);
    pt.x = q[i](0) + q_grad[i](0);
    pt.y = q[i](1) + q_grad[i](1);
    pt.z = q[i](2) + q_grad[i](2);
    mk.points.push_back(pt);

    marker_array.markers.push_back(mk);
  }

  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  // mk.action = visualization_msgs::Marker::DELETE;
  mk.id = q.size();
  // pubs_[pub_id].publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.color.a = 1.0;

  mk.scale.x = 0.2;
  mk.scale.y = 0.2;
  mk.scale.z = 0.2;

  geometry_msgs::Point pt;
  for (size_t i = 0; i < q.size(); ++i) {
    pt.x = q[i](0);
    pt.y = q[i](1);
    pt.z = q[i](2);
    mk.points.push_back(pt);
  }

  marker_array.markers.push_back(mk);

  vis_pub_.publish(marker_array);
}
} // namespace fast_planner