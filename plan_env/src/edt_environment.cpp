#include <plan_env/edt_environment.h>

namespace fast_planner {
/* ============================== edt_environment ==============================
 */
void EDTEnvironment::init() {}

void EDTEnvironment::setMap(MapServer::Ptr &map_server) {
  this->map_server_ = map_server;
  resolution_inv_ = 1.0 / map_server->getResolution();
}

void EDTEnvironment::setObjPrediction(ObjPrediction prediction) {
  this->obj_prediction_ = prediction;
}

void EDTEnvironment::setObjScale(ObjScale scale) { this->obj_scale_ = scale; }

double EDTEnvironment::distToBox(int idx, const Eigen::Vector3d &pos, const double &time) {
  // Eigen::Vector3d pos_box = obj_prediction_->at(idx).evaluate(time);
  Eigen::Vector3d pos_box = obj_prediction_->at(idx).evaluateConstVel(time);

  Eigen::Vector3d box_max = pos_box + 0.5 * obj_scale_->at(idx);
  Eigen::Vector3d box_min = pos_box - 0.5 * obj_scale_->at(idx);

  Eigen::Vector3d dist;

  for (int i = 0; i < 3; i++) {
    dist(i) = pos(i) >= box_min(i) && pos(i) <= box_max(i)
                  ? 0.0
                  : min(fabs(pos(i) - box_min(i)), fabs(pos(i) - box_max(i)));
  }

  return dist.norm();
}

double EDTEnvironment::minDistToAllBox(const Eigen::Vector3d &pos, const double &time) {
  double dist = 10000000.0;
  for (int i = 0; i < (int)obj_prediction_->size(); i++) {
    double di = distToBox(i, pos, time);
    if (di < dist)
      dist = di;
  }

  return dist;
}

void EDTEnvironment::getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2]) {
  for (int x = 0; x < 2; x++)
    for (int y = 0; y < 2; y++)
      for (int z = 0; z < 2; z++) {
        // dists[x][y][z] = sdf_map_->getDistance(pts[x][y][z]);
        dists[x][y][z] = map_server_->getESDF()->getDistance(
            map_server_->getESDF()->positionToIndex(pts[x][y][z]));
      }
}

void EDTEnvironment::interpolateTrilinear(double values[2][2][2], const Eigen::Vector3d &diff,
                                          double &value, Eigen::Vector3d &grad) {
  // trilinear interpolation
  double v00 = (1 - diff(0)) * values[0][0][0] + diff(0) * values[1][0][0];
  double v01 = (1 - diff(0)) * values[0][0][1] + diff(0) * values[1][0][1];
  double v10 = (1 - diff(0)) * values[0][1][0] + diff(0) * values[1][1][0];
  double v11 = (1 - diff(0)) * values[0][1][1] + diff(0) * values[1][1][1];
  double v0 = (1 - diff(1)) * v00 + diff(1) * v10;
  double v1 = (1 - diff(1)) * v01 + diff(1) * v11;

  value = (1 - diff(2)) * v0 + diff(2) * v1;

  grad[2] = (v1 - v0) * resolution_inv_;
  grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * resolution_inv_;
  grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
  grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
  grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
  grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
  grad[0] *= resolution_inv_;
}

void EDTEnvironment::evaluateEDTWithGrad(const Eigen::Vector3d &pos, double time, double &dist,
                                         Eigen::Vector3d &grad) {
  map_server_->getESDF()->getDistanceAndGradient(pos, dist, grad);
}

double EDTEnvironment::evaluateCoarseEDT(Eigen::Vector3d &pos, double time) {
  double d1 = map_server_->getESDF()->getDistance(map_server_->getESDF()->positionToIndex(pos));
  if (time < 0.0) {
    return d1;
  } else {
    double d2 = minDistToAllBox(pos, time);
    return min(d1, d2);
  }
}

void EDTEnvironment::getFeaturesInFovDepth(const Eigen::Vector3d &pos,
                                           std::vector<Eigen::Vector3d> &res) {
  map_server_->getFeatureMap()->getFeatures(pos, res);
}

void EDTEnvironment::getFeaturesIndexInFovDepth(const Eigen::Vector3d &pos, std::vector<int> &res) {
  map_server_->getFeatureMap()->getFeaturesIndex(pos, res);
}

void EDTEnvironment::getFeaturesUsingIndex(const std::vector<int> &index,
                                           std::vector<Eigen::Vector3d> &res) {
  map_server_->getFeatureMap()->indexToFeatures(index, res);
}
} // namespace fast_planner