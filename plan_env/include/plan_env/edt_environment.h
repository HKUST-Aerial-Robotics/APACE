#ifndef _EDT_ENVIRONMENT_H_
#define _EDT_ENVIRONMENT_H_

#include "plan_env/obj_predictor.h"
#include "voxel_mapping/map_server.h"

#include <Eigen/Eigen>
#include <iostream>
#include <utility>

using std::cout;
using std::endl;
using std::list;
using std::pair;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

using voxel_mapping::MapServer;

namespace fast_planner {

class EDTEnvironment {
public:
  typedef shared_ptr<EDTEnvironment> Ptr;
  typedef shared_ptr<const EDTEnvironment> ConstPtr;

  EDTEnvironment(/* args */) {}
  ~EDTEnvironment() {}

  void init();
  void setMap(MapServer::Ptr &map_server);
  void setObjPrediction(ObjPrediction prediction);
  void setObjScale(ObjScale scale);
  void evaluateEDTWithGrad(const Eigen::Vector3d &pos, double time, double &dist,
                           Eigen::Vector3d &grad);
  double evaluateCoarseEDT(Eigen::Vector3d &pos, double time);
  void getFeaturesInFovDepth(const Eigen::Vector3d &pos, std::vector<Eigen::Vector3d> &res);
  void getFeaturesIndexInFovDepth(const Eigen::Vector3d &pos, std::vector<int> &res);
  void getFeaturesUsingIndex(const std::vector<int> &index, std::vector<Eigen::Vector3d> &res);

  // deprecated
  void getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2]);
  void interpolateTrilinear(double values[2][2][2], const Eigen::Vector3d &diff, double &value,
                            Eigen::Vector3d &grad);

  voxel_mapping::MapServer::Ptr map_server_;

private:
  /* data */
  ObjPrediction obj_prediction_;
  ObjScale obj_scale_;
  double resolution_inv_;
  double distToBox(int idx, const Eigen::Vector3d &pos, const double &time);
  double minDistToAllBox(const Eigen::Vector3d &pos, const double &time);
};

} // namespace fast_planner

#endif