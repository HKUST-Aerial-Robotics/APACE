#ifndef _PARALLAX_UTIL_H_
#define _PARALLAX_UTIL_H_

#include "bspline/non_uniform_bspline.h"
#include "plan_env/edt_environment.h"

using namespace std;

namespace fast_planner {

enum WeightingPolicy { UNIFORM, DISTANCE, VERTICAL_VISIBILITY };

struct ParallaxConfig {
  double estimator_freq_; // frequency of consecutive frames in hz
  double max_parallax_;   // max parallax angle between consecutive frames in rad
  double pot_a_;          // potential func: a(x-max_parallax_)^2
};

class ParallaxUtil {
private:
  ParallaxConfig config_;       // Parallax util parameters config
  EDTEnvironment::Ptr edt_env_; // Environment and mapping API

public:
  ParallaxUtil(){};
  ParallaxUtil(const ros::NodeHandle &nh);
  ParallaxUtil(ParallaxConfig config);
  ~ParallaxUtil(){};
  void setEDTEnvironment(const EDTEnvironment::Ptr &edt);

  // Parallax
  void calcParaValueAndGradients(const Eigen::Vector3d v1, const Eigen::Vector3d v2,
                                 double &parallax, bool calc_grad, Eigen::Vector3d &dpara_dv1,
                                 Eigen::Vector3d &dpara_dv2);
  void calcParaPotentialAndGradients(const double parallax, const double dt, double &para_pot,
                                     double &dpot_dpara);
  void calcParaCostAndGradientsKnots(const vector<Eigen::Vector3d> q, const double dt,
                                     const vector<Eigen::Vector3d> features, double &pot_cost,
                                     vector<Eigen::Vector3d> &dpot_dq);
  // Vertical visibility & covisibility
  void calcVVValueAndGradients(const Eigen::Vector3d a, const Eigen::Vector3d b, double &cos_theta,
                               bool calc_grad, Eigen::Vector3d &dcos_theta_da,
                               Eigen::Vector3d &dcos_theta_db);
  void calcVVPotentialAndGradients(const double cos_theta, double &cos_theta_pot,
                                   double &dpot_dcos_theta);
  // void calcVVValueAndGradients2(const Eigen::Vector3d a, const Eigen::Vector3d b, double &theta,
  //                              bool calc_grad, Eigen::Vector3d &dtheta_da,
  //                              Eigen::Vector3d &dtheta_db);
  // void calcVVPotentialAndGradients2(const double theta, double &theta_pot,
  //                                  double &dpot_dtheta);
  void calcVVCostAndGradientsKnots(const vector<Eigen::Vector3d> q,
                                   const vector<Eigen::Vector3d> features, const double &knot_span,
                                   double &pot_cost, vector<Eigen::Vector3d> &dpot_dq);
  void calcVCVCostAndGradientsKnots(const vector<Eigen::Vector3d> q,
                                    const vector<Eigen::Vector3d> features, const double &knot_span,
                                    double &pot_cost, vector<Eigen::Vector3d> &dpot_dq);
  void calcYawCVCostAndGradientsKnots(const vector<Eigen::Vector3d> q,
                                      const vector<Eigen::Vector3d> knots_pos,
                                      const vector<Eigen::Vector3d> knots_acc,
                                      const vector<Eigen::Vector3d> features, double &pot_cost,
                                      vector<Eigen::Vector3d> &dpot_dq);

  // Utils
  double weightingPolicy(const WeightingPolicy &policy,
                         const Eigen::Vector3d &knot = Eigen::Vector3d::Zero(),
                         const Eigen::Vector3d &f = Eigen::Vector3d::Zero(),
                         const Eigen::Vector3d &thrust_dir = Eigen::Vector3d::Zero());
  Eigen::Vector3d getThrustDirection(const Eigen::Vector3d &q1, const Eigen::Vector3d &q2,
                                     const Eigen::Vector3d &q3, const double &knot_span);

private: // TODO: move helper function into private
};
} // namespace fast_planner
#endif