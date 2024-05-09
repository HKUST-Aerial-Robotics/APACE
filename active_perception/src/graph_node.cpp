#include <active_perception/graph_node.h>

namespace fast_planner {
// Static data
double ViewNode::vm_;
double ViewNode::am_;
double ViewNode::yd_;
double ViewNode::ydd_;
double ViewNode::w_dir_;
Astar::Ptr ViewNode::astar_;
RayCaster::Ptr ViewNode::caster_;
MapServer::Ptr ViewNode::map_server_;

// Graph node for viewpoints planning
ViewNode::ViewNode(const Vector3d &p, const double &y) {
  pos_ = p;
  yaw_ = y;
  parent_ = nullptr;
  vel_.setZero(); // vel is zero by default, should be set explicitly
}

double ViewNode::costTo(const ViewNode::Ptr &node) {
  vector<Vector3d> path;
  double c = ViewNode::computeCost(pos_, node->pos_, yaw_, node->yaw_, vel_, yaw_dot_, path);
  // std::cout << "cost from " << id_ << " to " << node->id_ << " is: " << c << std::endl;
  return c;
}

double ViewNode::searchPath(const Vector3d &p1, const Vector3d &p2, vector<Vector3d> &path) {
  // Try connect two points with straight line
  bool safe = true;
  Vector3i idx;
  caster_->input(p1, p2);
  while (caster_->nextId(idx)) {
    if (map_server_->getOccupancyGrid()->getVoxel(idx).value ==
            voxel_mapping::OccupancyType::OCCUPIED ||
        map_server_->getOccupancyGrid()->getVoxel(idx).value ==
            voxel_mapping::OccupancyType::UNKNOWN ||
        !map_server_->getTSDF()->isInBox(idx)) {
      safe = false;
      break;
    }
  }
  if (safe) {
    path = {p1, p2};
    return (p1 - p2).norm();
  }
  // Search a path using decreasing resolution
  // vector<double> res = {0.2};
  // for (int k = 0; k < res.size(); ++k) {
  astar_->reset();
  // astar_->setResolution(res[k]);
  if (astar_->search(p1, p2) == Astar::REACH_END) {
    path = astar_->getPath();
    return astar_->pathLength(path);
  }
  // }
  // Use Astar early termination cost as an estimate
  path = {p1, p2};
  return 1000.0 + (p1 - p2).norm();
}

// Input:
// p1, p2: start and end position
// y1, y2: start and end yaw
// v1: start velocity
// yd1: start yaw rate
// Output:
// path: path from p1 to p2
double ViewNode::computeCost(const Vector3d &p1, const Vector3d &p2, const double &y1,
                             const double &y2, const Vector3d &v1, const double &yd1,
                             vector<Vector3d> &path, bool verbose) {
  // Cost of position change
  double pos_cost = ViewNode::searchPath(p1, p2, path) / vm_;

  if (verbose) {
    std::cout << "path length: " << pos_cost << std::endl;
    std::cout << "path: " << std::endl;
    for (int k = 0; k < path.size(); ++k)
      std::cout << path[k].transpose() << " | ";
  }

  // Consider velocity change
  if (v1.norm() > 1e-3) {
    Vector3d dir;
    if (path.size() < 2)
      dir = (p2 - p1).normalized();
    else
      dir = (path[1] - path[0]).normalized();
    double vc = v1.dot(dir);
    double t = pow(vm_ - vc, 2) / (2 * vm_ * am_);
    if (vc < 0)
      t += 2 * fabs(vc) / am_;
    pos_cost += w_dir_ * t;

    if (verbose) {
      std::cout << "velocity change: " << w_dir_ * t << std::endl;
    }
  }

  // // Cost of yaw change
  // double diff = fabs(y2 - y1);
  // diff = min(diff, 2 * M_PI - diff);
  // double yaw_cost = diff / yd_;
  // Cost of yaw change
  double diff = fabs(y2 - y1);
  diff = min(diff, 2 * M_PI - diff);
  double yaw_cost = 0.0;
  if (diff < 0.5 * yd_ * yd_ / ydd_)
    yaw_cost = sqrt(2 * diff / ydd_);
  else
    yaw_cost = yd_ / ydd_ + (diff - 0.5 * yd_ * yd_ / ydd_) / yd_;

  if (verbose) {
    std::cout << "pos_cost: " << pos_cost << ", "
              << "yaw_cost: " << yaw_cost << std::endl;
  }

  return max(pos_cost, yaw_cost);

  // // Consider yaw rate change
  // if (fabs(yd1) > 1e-3)
  // {
  //   double diff1 = y2 - y1;
  //   while (diff1 < -M_PI)
  //     diff1 += 2 * M_PI;
  //   while (diff1 > M_PI)
  //     diff1 -= 2 * M_PI;
  //   double diff2 = diff1 > 0 ? diff1 - 2 * M_PI : 2 * M_PI + diff1;
  // }
  // else
  // {
  // }
}
} // namespace fast_planner