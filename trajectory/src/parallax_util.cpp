#include "parallax_util.h"

namespace fast_planner {
ParallaxUtil::ParallaxUtil(const ros::NodeHandle &nh) {
  nh.param("parallax/estimator_freq", config_.estimator_freq_, -1.0);
  nh.param("parallax/max_parallax", config_.max_parallax_, -1.0);
  nh.param("parallax/pot_a", config_.pot_a_, -1.0);
}

ParallaxUtil::ParallaxUtil(ParallaxConfig config) : config_(config) {}

void ParallaxUtil::setEDTEnvironment(const EDTEnvironment::Ptr &edt) { edt_env_ = edt; }

void ParallaxUtil::calcParaValueAndGradients(const Eigen::Vector3d v1, const Eigen::Vector3d v2,
                                             double &parallax, bool calc_grad,
                                             Eigen::Vector3d &dpara_dv1,
                                             Eigen::Vector3d &dpara_dv2) {
  parallax = acos(v1.dot(v2) / (v1.norm() * v2.norm()));

  // Calculate gradients dpara_dv1 and dpara_dv2
  if (calc_grad) {
    double v1_norm_inv = 1 / v1.norm();
    double v2_norm_inv = 1 / v2.norm();

    // Use chain rule
    double u = v1.dot(v2) / (v1.norm() * v2.norm());
    double dpara_du = -1 / sqrt(1 - pow(u, 2));

    // Compute directly
    Eigen::Vector3d du_dv1 =
        v2_norm_inv * (v1_norm_inv * v2 - v1.dot(v2) * pow(v1_norm_inv, 3) * v1);
    Eigen::Vector3d du_dv2 =
        v1_norm_inv * (v2_norm_inv * v1 - v1.dot(v2) * pow(v2_norm_inv, 3) * v2);

    dpara_dv1 = dpara_du * du_dv1;
    dpara_dv2 = dpara_du * du_dv2;
  }
}

void ParallaxUtil::calcParaPotentialAndGradients(const double parallax, const double dt,
                                                 double &para_pot, double &dpot_dpara) {
  // Potential func: f(x) = 0 if x < max; f(x) = a(x-max)^2 otherwise
  double max_para_btw_knots = config_.max_parallax_ * config_.estimator_freq_ * dt;
  // ROS_INFO("max_para_btw_knots: %f", max_para_btw_knots);
  if (parallax < max_para_btw_knots) {
    para_pot = 0;
    dpot_dpara = 0;
  } else {
    para_pot = config_.pot_a_ * pow(parallax - max_para_btw_knots, 2);
    dpot_dpara = 2 * config_.pot_a_ * (parallax - max_para_btw_knots);
  }
}

void ParallaxUtil::calcParaCostAndGradientsKnots(const vector<Eigen::Vector3d> q, const double dt,
                                                 const vector<Eigen::Vector3d> features,
                                                 double &pot_cost,
                                                 vector<Eigen::Vector3d> &dpot_dq) {
  CHECK_EQ(q.size(), 4) << "Control points set should have exactly 4 points!";

  pot_cost = 0;
  dpot_dq.clear();
  for (int i = 0; i < 4; i++)
    dpot_dq.push_back(Eigen::Vector3d::Zero());

  Eigen::Vector3d knot1 = 1 / 6.0 * (q[0] + 4 * q[1] + q[2]);
  Eigen::Vector3d knot2 = 1 / 6.0 * (q[1] + 4 * q[2] + q[3]);
  Eigen::Vector3d knot_mid = 0.5 * (knot1 + knot2);
  Eigen::Vector3d thrust_dir1 = getThrustDirection(q[0], q[1], q[2], dt);
  Eigen::Vector3d thrust_dir2 = getThrustDirection(q[1], q[2], q[3], dt);
  Eigen::Vector3d thrust_mid = 0.5 * (thrust_dir1 + thrust_dir2);

  int num_features = features.size();
  double total_weight = 0.0;
  TicToc tic;
  for (Eigen::Vector3d f : features) {
    Eigen::Vector3d v1 = knot1 - f;
    Eigen::Vector3d v2 = knot2 - f;

    double parallax;
    Eigen::Vector3d dpara_dv1, dpara_dv2;
    calcParaValueAndGradients(v1, v2, parallax, true, dpara_dv1, dpara_dv2);

    double para_pot, dpot_dpara;
    calcParaPotentialAndGradients(parallax, dt, para_pot, dpot_dpara);

    double w = weightingPolicy(VERTICAL_VISIBILITY, knot_mid, f, thrust_mid);
    // double w = weightingPolicy(UNIFORM);
    pot_cost = (pot_cost * total_weight + para_pot * w) / (total_weight + w);

    vector<Eigen::Vector3d> dpot_dq_cur;
    dpot_dq_cur.push_back(w * dpot_dpara * dpara_dv1 * (1 / 6.0));
    dpot_dq_cur.push_back(w * dpot_dpara * (dpara_dv1 * (4 / 6.0) + dpara_dv2 * (1 / 6.0)));
    dpot_dq_cur.push_back(w * dpot_dpara * (dpara_dv1 * (1 / 6.0) + dpara_dv2 * (4 / 6.0)));
    dpot_dq_cur.push_back(w * dpot_dpara * dpara_dv2 * (1 / 6.0));

    for (int i = 0; i < 4; i++) {
      dpot_dq[i] = (dpot_dq[i] * total_weight + dpot_dq_cur[i]) / (total_weight + w);
    }

    total_weight += w;
  }
  // ROS_INFO("Parallax takes %f ms for %d features", tic.toc()*1000, num_features);
}

void ParallaxUtil::calcVVValueAndGradients(const Eigen::Vector3d a, const Eigen::Vector3d b,
                                           double &cos_theta, bool calc_grad,
                                           Eigen::Vector3d &dcos_theta_da,
                                           Eigen::Vector3d &dcos_theta_db) {
  cos_theta = a.dot(b) / (a.norm() * b.norm());

  // Calculate gradients dcos_theta_da and dcos_theta_db
  if (calc_grad) {
    double a_norm_inv = 1 / a.norm();
    double b_norm_inv = 1 / b.norm();

    // Compute directly
    dcos_theta_da = b_norm_inv * (a_norm_inv * b - a.dot(b) * pow(a_norm_inv, 3) * a);
    dcos_theta_db = a_norm_inv * (b_norm_inv * a - a.dot(b) * pow(b_norm_inv, 3) * b);
  }
}
void ParallaxUtil::calcVVPotentialAndGradients(const double cos_theta, double &cos_theta_pot,
                                               double &dpot_dcos_theta) {
  double max = cos(M_PI / 3.0);
  double min = cos(M_PI - M_PI / 3.0);
  // a (M_PI / 3.0)^2 = k
  // double a = 1.0 / (pow(M_PI / 3.0, 2));
  // double a = 10.0 / (pow(M_PI / 3.0, 2));
  double a = 10;
  if (cos_theta > max) {
    cos_theta_pot = a * pow(cos_theta - max, 2);
    dpot_dcos_theta = 2 * a * (cos_theta - max);
  } else if (cos_theta < min) {
    cos_theta_pot = a * pow(cos_theta - min, 2);
    dpot_dcos_theta = 2 * a * (cos_theta - min);
  } else {
    cos_theta_pot = 0;
    dpot_dcos_theta = 0;
  }
}

void ParallaxUtil::calcVVCostAndGradientsKnots(const vector<Eigen::Vector3d> q,
                                               const vector<Eigen::Vector3d> features,
                                               const double &knot_span, double &pot_cost,
                                               vector<Eigen::Vector3d> &dpot_dq) {
  CHECK_EQ(q.size(), 3) << "Control points set should have exactly 3 points!";

  pot_cost = 0;
  dpot_dq.clear();
  for (int i = 0; i < 3; i++)
    dpot_dq.push_back(Eigen::Vector3d::Zero());

  double knot_span_inv2 = 1 / pow(knot_span, 2);
  Eigen::Vector3d acc = knot_span_inv2 * (q[0] - 2 * q[1] + q[2]);
  Eigen::Vector3d gravity(0, 0, -9.81);
  Eigen::Vector3d a = acc - gravity; // thrust
  Eigen::Vector3d knot = 1 / 6.0 * (q[0] + 4 * q[1] + q[2]);

  Eigen::Vector3d da_dq(knot_span_inv2, -2 * knot_span_inv2, knot_span_inv2);
  Eigen::Vector3d db_dq(-1 / 6.0, -4 / 6.0, -1 / 6.0);

  double total_weight = 0.0;
  for (Eigen::Vector3d f : features) {
    Eigen::Vector3d b = f - knot;
    double w = weightingPolicy(UNIFORM, knot, f);

    double cos_theta;
    Eigen::Vector3d dcos_theta_da, dcos_theta_db;
    calcVVValueAndGradients(a, b, cos_theta, true, dcos_theta_da, dcos_theta_db);

    double cos_theta_pot, dpot_dcos_theta;
    calcVVPotentialAndGradients(cos_theta, cos_theta_pot, dpot_dcos_theta);

    pot_cost = (pot_cost * total_weight + cos_theta_pot * w) / (total_weight + w);

    for (int i = 0; i < 3; i++) {
      Eigen::Vector3d dpot_dq_cur =
          w * dpot_dcos_theta * (dcos_theta_da * da_dq[i] + dcos_theta_db * db_dq[i]);
      dpot_dq[i] = (dpot_dq[i] * total_weight + dpot_dq_cur) / (total_weight + w);
    }

    total_weight += w;
  }
}

void ParallaxUtil::calcVCVCostAndGradientsKnots(const vector<Eigen::Vector3d> q,
                                                const vector<Eigen::Vector3d> features,
                                                const double &knot_span, double &pot_cost,
                                                vector<Eigen::Vector3d> &dpot_dq) {
  CHECK_EQ(q.size(), 4) << "Control points set should have exactly 4 points!";

  pot_cost = 0;
  dpot_dq.clear();
  for (int i = 0; i < 4; i++)
    dpot_dq.push_back(Eigen::Vector3d::Zero());

  double knot_span_inv2 = 1 / pow(knot_span, 2);
  Eigen::Vector3d gravity(0, 0, -9.81);

  double total_weight = 0.0;
  for (Eigen::Vector3d f : features) {
    double w = weightingPolicy(UNIFORM);

    vector<double> cos_theta_pot_vec;
    vector<vector<Eigen::Vector3d>> dpoti_dqj_vec;

    // Calculate visibility cost and gradients for each knot
    for (int i = 0; i < 2; i++) {
      // Calculate vector a,b and their gradients
      Eigen::Vector3d acc = knot_span_inv2 * (q[i] - 2 * q[i + 1] + q[i + 2]);
      Eigen::Vector3d a = acc - gravity; // thrust
      Eigen::Vector3d knot = 1 / 6.0 * (q[i] + 4 * q[i + 1] + q[i + 2]);
      Eigen::Vector3d b = f - knot;

      Eigen::Vector3d da_dq_temp(knot_span_inv2, -2 * knot_span_inv2, knot_span_inv2);
      Eigen::Vector4d da_dq = Eigen::Vector4d::Zero();
      da_dq.segment(i, 3) = da_dq_temp;
      Eigen::Vector3d db_dq_temp(-1 / 6.0, -4 / 6.0, -1 / 6.0);
      Eigen::Vector4d db_dq = Eigen::Vector4d::Zero();
      db_dq.segment(i, 3) = db_dq_temp;

      // Calculate cos_theta
      double cos_theta;
      Eigen::Vector3d dcos_theta_da, dcos_theta_db;
      calcVVValueAndGradients(a, b, cos_theta, true, dcos_theta_da, dcos_theta_db);

      // Calculate potential cost function
      double cos_theta_pot, dpot_dcos_theta;
      calcVVPotentialAndGradients(cos_theta, cos_theta_pot, dpot_dcos_theta);

      cos_theta_pot_vec.push_back(cos_theta_pot);

      // Calculate gradients of potential cost
      vector<Eigen::Vector3d> dpot_dqj;
      for (int j = 0; j < 4; j++) {
        Eigen::Vector3d dpoti_dqj =
            w * dpot_dcos_theta * (dcos_theta_da * da_dq[j] + dcos_theta_db * db_dq[j]);
        dpot_dqj.push_back(dpoti_dqj);
      }
      dpoti_dqj_vec.push_back(dpot_dqj);
    }

    // Calculate co-visbility potential cost function and its gradient
    // f_cov(theta_1, theta_2) = (f_v(theta_1) + 1)(f_v(theta_2) + 1) - 1
    double covisib_pot = (cos_theta_pot_vec[0] + 1) * (cos_theta_pot_vec[1] + 1) - 1;
    pot_cost = (pot_cost * total_weight + covisib_pot * w) / (total_weight + w);

    for (int j = 0; j < 4; j++) {
      Eigen::Vector3d dcovisb_pot_dq_cur = dpoti_dqj_vec[0][j] * (cos_theta_pot_vec[1] + 1) +
                                           (cos_theta_pot_vec[0] + 1) * dpoti_dqj_vec[1][j];
      dpot_dq[j] = (dpot_dq[j] * total_weight + dcovisb_pot_dq_cur) / (total_weight + w);
    }

    total_weight += w;
  }
}

void ParallaxUtil::calcYawCVCostAndGradientsKnots(const vector<Eigen::Vector3d> q,
                                                  const vector<Eigen::Vector3d> knots_pos,
                                                  const vector<Eigen::Vector3d> knots_acc,
                                                  const vector<Eigen::Vector3d> features,
                                                  double &pot_cost,
                                                  vector<Eigen::Vector3d> &dpot_dq) {
  CHECK_EQ(q.size(), 4) << "Control points set should have exactly 4 points!";

  pot_cost = 0;
  dpot_dq.clear();
  for (int i = 0; i < 4; i++)
    dpot_dq.push_back(Eigen::Vector3d::Zero());

  Eigen::Vector3d gravity(0, 0, -9.81);
  double total_weight = 0.0;
  for (Eigen::Vector3d f : features) {
    double w = weightingPolicy(UNIFORM);

    vector<double> v3_theta3_vec, v1v2_vec;
    vector<vector<Eigen::Vector3d>> dv3_dyaw_vec;

    // Calculate visibility cost and gradients for each knot
    for (int i = 0; i < 2; i++) {
      Eigen::Vector3d yaw_knot = 1 / 6.0 * (q[i] + 4 * q[i + 1] + q[i + 2]);
      double yaw = yaw_knot(0);

      // Calculate vectors n1, ny, n3 ,b and their gradients
      Eigen::Vector3d n1, ny, n3, n2, b;
      n1 = knots_acc[i] - gravity; // thrust
      ny << cos(yaw), sin(yaw), 0;
      n3 = n1.cross(ny);
      n2 = n3.cross(n1);
      b = f - knots_pos[i];

      Eigen::Vector3d dn3_dyaw;
      dn3_dyaw << -n1(2) * cos(yaw), -n1(2) * sin(yaw), n1(1) * sin(yaw) + n1(0) * cos(yaw);

      Eigen::Vector4d dyaw_dq = Eigen::Vector4d::Zero();
      dyaw_dq.segment(i, 3) = Eigen::Vector3d(1 / 6.0, 4 / 6.0, 1 / 6.0);

      // Calculate v1 * v2
      double sin_theta1, v1_theta1, cos_theta2, v2_theta2, v1v2;
      double k1 = 40;
      double k2 = 10;
      double fov_vertical = M_PI / 3.0;
      sin_theta1 = n1.cross(b).norm() / (n1.norm() * b.norm());
      v1_theta1 = 1 / (1 + exp(-k1 * (sin_theta1 - sin((M_PI - fov_vertical) / 2.0))));
      cos_theta2 = n2.dot(b) / (n2.norm() * b.norm());
      v2_theta2 = 1 / (1 + std::exp(-k2 * cos_theta2));
      v1v2 = v1_theta1 * v2_theta2;

      // Calculate v3(theta3) and gradients
      double sin_theta3, v3_theta3;
      double k3 = 20;
      double fov_horizontal = M_PI / 2.0;
      sin_theta3 = n3.cross(b).norm() / (n3.norm() * b.norm());
      v3_theta3 = 1 / (1 + exp(-k3 * (sin_theta3 - sin((M_PI - fov_horizontal) / 2.0))));

      Eigen::Vector3d dsin_theta3_dn3;
      Eigen::Vector3d c = n3.cross(b);
      double n3_norm, b_norm, c_norm;
      n3_norm = n3.norm();
      b_norm = b.norm();
      c_norm = c.norm();
      dsin_theta3_dn3 = (pow(-n3_norm, 2) * b.cross(c) - pow(c_norm, 2) * n3) /
                        (pow(n3_norm, 3) * b_norm * c_norm);
      double dv3_dsin_theta3 =
          k3 * exp(-k3 * (sin_theta3 - sin((M_PI - fov_horizontal) / 2.0))) * pow(v3_theta3, 2);

      // Combine gradients using chain rule
      double dv3_dyaw = dv3_dsin_theta3 * dsin_theta3_dn3.dot(dn3_dyaw);

      // Store results
      v1v2_vec.push_back(v1v2);
      v3_theta3_vec.push_back(v3_theta3);
      vector<Eigen::Vector3d> dv3_dyaw_i;
      for (int j = 0; j < 4; j++) {
        Eigen::Vector3d dv3_dqj = Eigen::Vector3d(dv3_dyaw, 0, 0) * dyaw_dq[j];
        dv3_dyaw_i.push_back(dv3_dqj);
      }
      dv3_dyaw_vec.push_back(dv3_dyaw_i);
    }

    // Calculate co-visbility potential cost function and its gradient
    double covisibility_cost = -(v1v2_vec[0] * v1v2_vec[1] * v3_theta3_vec[0] * v3_theta3_vec[1]);
    pot_cost = (pot_cost * total_weight + covisibility_cost * w) / (total_weight + w);

    for (int j = 0; j < 4; j++) {
      Eigen::Vector3d dcovisb_pot_dq_cur =
          -v1v2_vec[0] * v1v2_vec[1] *
          (dv3_dyaw_vec[0][j] * v3_theta3_vec[1] + v3_theta3_vec[0] * dv3_dyaw_vec[1][j]);
      dpot_dq[j] = (dpot_dq[j] * total_weight + dcovisb_pot_dq_cur) / (total_weight + w);
    }

    total_weight += w;
  }
}
double ParallaxUtil::weightingPolicy(const WeightingPolicy &policy, const Eigen::Vector3d &knot,
                                     const Eigen::Vector3d &f, const Eigen::Vector3d &thrust_dir) {
  double weight = 0.0;
  if (policy == UNIFORM)
    weight = 1.0;
  else if (policy == DISTANCE) {
    double dist = (knot - f).norm();
    weight = dist > 1.0 ? 1 / pow(dist, 2) : 1.0;
  } else if (policy == VERTICAL_VISIBILITY) {
    Eigen::Vector3d v = f - knot;
    double sin_theta = v.cross(thrust_dir).norm() / v.norm();
    // double sin_alpha = sin(M_PI / 4.0);
    // double w = 1.0 / (1 + exp(-20 * (sin_theta - sin_alpha)));
    double fov_vertical = M_PI / 3.0;
    double sin_alpha = sin((M_PI - fov_vertical) / 2.0);
    double w = 1.0 / (1 + exp(-60 * (sin_theta - sin_alpha)));
    weight = w;
  }
  return weight;
}

Eigen::Vector3d ParallaxUtil::getThrustDirection(const Eigen::Vector3d &q1,
                                                 const Eigen::Vector3d &q2,
                                                 const Eigen::Vector3d &q3,
                                                 const double &knot_span) {
  // Acc trajectory is a b-spline curve with p=1 -> knot points are control points
  // Q_i = p * (P_{i+1} - P_i) / (u_{i+p+1} - u_{i+1})
  // R_i = (p-1) * (Q_{i+1} - Q_i) / (u_{i+p} - u_{i+1})
  Eigen::Vector3d acc = 1 / pow(knot_span, 2) * (q3 - 2 * q2 + q1);
  Eigen::Vector3d gravity(0, 0, -9.81);
  Eigen::Vector3d thrust_dir = (acc - gravity).normalized();
  return thrust_dir;
}

/* ------------------------------- Deprecated ------------------------------- */
#ifdef DEBUG
void ParallaxUtil::calcVVValueAndGradients(const Eigen::Vector3d a, const Eigen::Vector3d b,
                                           double &sin_theta, bool calc_grad,
                                           Eigen::Vector3d &dsin_theta_da,
                                           Eigen::Vector3d &dsin_theta_db) {
  Eigen::Vector3d c = a.cross(b);
  double a_norm = a.norm();
  double b_norm = b.norm();
  double c_norm = c.norm();
  sin_theta = c_norm / (a_norm * b_norm);

  if (calc_grad) {
    // Compute directly
    dsin_theta_da =
        (-pow(a_norm, 2) * b.cross(c) - pow(c_norm, 2) * a) / (pow(a_norm, 3) * b_norm * c_norm);
    dsin_theta_db =
        (pow(b_norm, 2) * a.cross(c) - pow(c_norm, 2) * b) / (pow(b_norm, 3) * a_norm * c_norm);
  }
}

void ParallaxUtil::calcVVPotentialAndGradients(const double sin_theta, double &sin_theta_pot,
                                               double &dpot_dsin_theta) {
  // Sigmoid potential func: v(\theta) = 1/(1+e^{-k(sin\theta - sin\alpha)})
  double sin_alpha = sin(M_PI / 4);
  double k = 5;
  double exp_abbre = exp(-k * (sin_theta - sin_alpha));
  sin_theta_pot = 1 - 1 / (1 + exp_abbre);
  dpot_dsin_theta = -(k * exp_abbre) / pow(1 + exp_abbre, 2);
  // sin_theta_pot = 1 / (1 + exp_abbre);
  // dpot_dsin_theta = (k * exp_abbre) / pow(1 + exp_abbre, 2);

  // Linear potential func: v(\theta) = a*(sin\alpha - sin\theta)^2, if sin\theta < sin\alpha
  // double sin_alpha = sin(M_PI / 4);
  // double a = 20;
  // if(sin_theta < sin_alpha){
  //   sin_theta_pot = a * pow(sin_alpha - sin_theta, 2);
  //   dpot_dsin_theta = -2 * a * (sin_alpha - sin_theta);
  // }
}

void ParallaxUtil::calcVVValueAndGradients2(const Eigen::Vector3d a, const Eigen::Vector3d b,
                                            double &theta, bool calc_grad,
                                            Eigen::Vector3d &dtheta_da,
                                            Eigen::Vector3d &dtheta_db) {
  Eigen::Vector3d c = a.cross(b);
  double a_norm = a.norm();
  double b_norm = b.norm();
  double c_norm = c.norm();
  double sin_theta = c_norm / (a_norm * b_norm);
  double theta2 = asin(sin_theta);
  double dtheta2_dsin_theta = 1 / sqrt(1 - pow(sin_theta, 2));

  bool acute_angle = a.dot(b) > 0;
  theta = acute_angle ? theta2 : M_PI - theta2;
  double dtheta_dtheta2 = acute_angle ? 1 : -1;

  if (calc_grad) {
    // Compute directly
    Eigen::Vector3d dsin_theta_da =
        (-pow(a_norm, 2) * b.cross(c) - pow(c_norm, 2) * a) / (pow(a_norm, 3) * b_norm * c_norm);
    Eigen::Vector3d dsin_theta_db =
        (pow(b_norm, 2) * a.cross(c) - pow(c_norm, 2) * b) / (pow(b_norm, 3) * a_norm * c_norm);
    dtheta_da = dtheta_dtheta2 * dtheta2_dsin_theta * dsin_theta_da;
    dtheta_db = dtheta_dtheta2 * dtheta2_dsin_theta * dsin_theta_db;
  }
}

void ParallaxUtil::calcVVPotentialAndGradients2(const double theta, double &theta_pot,
                                                double &dpot_dtheta) {
  // Potential func:
  // f(x) = 0 if min < x < max;
  // f(x) = a(x-max)^2 if x > max;
  // f(x) = a(x-min)^2 if x < min;
  double min = M_PI / 3.0;
  double max = M_PI - min;
  // a (M_PI / 3.0)^2 = 1
  double a = 1.0 / (pow(M_PI / 3.0, 2));
  // double a = 10.0;
  if (theta > max) {
    theta_pot = a * pow(theta - max, 2);
    dpot_dtheta = 2 * a * (theta - max);
  } else if (theta < min) {
    theta_pot = a * pow(theta - min, 2);
    dpot_dtheta = 2 * a * (theta - min);
  } else {
    theta_pot = 0;
    dpot_dtheta = 0;
  }
}

void ParallaxUtil::calcVVCostAndGradientsKnots(const vector<Eigen::Vector3d> q,
                                               const vector<Eigen::Vector3d> features,
                                               const double &knot_span, double &pot_cost,
                                               vector<Eigen::Vector3d> &dpot_dq) {
  CHECK_EQ(q.size(), 3) << "Control points set should have exactly 3 points!";

  pot_cost = 0;
  dpot_dq.clear();
  for (int i = 0; i < 3; i++)
    dpot_dq.push_back(Eigen::Vector3d::Zero());

  double knot_span_inv2 = 1 / pow(knot_span, 2);
  Eigen::Vector3d knot = 1 / 6.0 * (q[0] + 4 * q[1] + q[2]);
  Eigen::Vector3d acc = knot_span_inv2 * (q[0] - 2 * q[1] + q[2]);
  Eigen::Vector3d gravity(0, 0, -9.81);
  Eigen::Vector3d a = acc - gravity; // thrust
  Eigen::Vector3d da_dq(knot_span_inv2, -2 * knot_span_inv2, knot_span_inv2);
  Eigen::Vector3d db_dq(-1 / 6.0, -4 / 6.0, -1 / 6.0);

  int num_features = features.size();
  double total_weight = 0.0;
  TicToc tic;
  double total_sin_theta = 0.0;
  int mood = 2;
  for (Eigen::Vector3d f : features) {
    Eigen::Vector3d b = f - knot;
    double w = weightingPolicy(UNIFORM, knot, f);

    if (mood == 1) {
      double sin_theta;
      Eigen::Vector3d dsin_theta_da, dsin_theta_db;
      calcVVValueAndGradients(a, b, sin_theta, true, dsin_theta_da, dsin_theta_db);
      total_sin_theta += sin_theta;

      double sin_theta_pot, dpot_dsin_theta;
      calcVVPotentialAndGradients(sin_theta, sin_theta_pot, dpot_dsin_theta);

      pot_cost = (pot_cost * total_weight + sin_theta_pot * w) / (total_weight + w);

      for (int i = 0; i < 3; i++) {
        Eigen::Vector3d dpot_dq_cur =
            w * dpot_dsin_theta * (dsin_theta_da * da_dq[i] + dsin_theta_db * db_dq[i]);
        dpot_dq[i] = (dpot_dq[i] * total_weight + dpot_dq_cur) / (total_weight + w);
      }
    } else if (mood == 2) {
      double theta;
      Eigen::Vector3d dtheta_da, dtheta_db;
      calcVVValueAndGradients2(a, b, theta, true, dtheta_da, dtheta_db);

      double theta_pot, dpot_dtheta;
      calcVVPotentialAndGradients2(theta, theta_pot, dpot_dtheta);

      pot_cost = (pot_cost * total_weight + theta_pot * w) / (total_weight + w);
      // ROS_INFO("theta: %f deg, pot_cost: %f", theta / M_PI * 180, pot_cost);

      for (int i = 0; i < 3; i++) {
        // Eigen::Vector3d dpot_dq_cur =
        //     w * dpot_dtheta * (dtheta_da * da_dq[i] + dtheta_db * db_dq[i]);
        Eigen::Vector3d dpot_dq_cur = w * dpot_dtheta * (dtheta_da * da_dq[i]);
        dpot_dq[i] = (dpot_dq[i] * total_weight + dpot_dq_cur) / (total_weight + w);
      }
    }

    total_weight += w;
  }
  total_sin_theta /= (double)num_features;
  // ROS_INFO("Average sin_theta: %f", total_sin_theta);
}
#endif
} // namespace fast_planner