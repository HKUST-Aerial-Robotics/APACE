#ifndef _GRAPH_NODE_H_
#define _GRAPH_NODE_H_

#include "pathfinding/astar.h"
#include "raycast/raycast.h"

#include <Eigen/Eigen>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <list>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>

using Eigen::Vector3d;
using Eigen::Vector3i;
using std::cout;
using std::list;
using std::queue;
using std::shared_ptr;
using std::unique_ptr;
using std::unordered_map;
using std::vector;

class RayCaster;

using voxel_mapping::MapServer;

namespace fast_planner {
// Basic noded type containing only general artributes required by graph search
class BaseNode {
public:
  typedef shared_ptr<BaseNode> Ptr;
  BaseNode() {
    g_value_ = 1000000;
    closed_ = false;
  }
  ~BaseNode() {}

  virtual void print() { std::cout << "Base node" << std::endl; }

  int id_;
  bool closed_;
  double g_value_;
};

// Node type for viewpoint refinement
class Astar;
class ViewNode : public BaseNode {
public:
  typedef shared_ptr<ViewNode> Ptr;
  ViewNode(const Vector3d &p, const double &y);
  ViewNode() {}
  ~ViewNode() {}

  virtual void print() { std::cout << "View node" << yaw_ << std::endl; }

  void printNeighbors() {
    for (auto v : neighbors_)
      v->print();
  }

  double costTo(const ViewNode::Ptr &node);
  static double computeCost(const Vector3d &p1, const Vector3d &p2, const double &y1,
                            const double &y2, const Vector3d &v1, const double &yd1,
                            vector<Vector3d> &path, bool verbose = false);
  static double computeCostBBox(const Vector3d &p1, const Vector3d &p2, const double &y1,
                                const double &y2, const Vector3d &v1, const double &yd1,
                                const Vector3d &bbox_min, const Vector3d &bbox_max,
                                vector<Vector3d> &path, bool verbose = false);
  static double computeCostUnknown(const Vector3d &p1, const Vector3d &p2, const double &y1,
                                   const double &y2, const Vector3d &v1, const double &yd1,
                                   vector<Vector3d> &path);
  static double computeCostUnknownBBox(const Vector3d &p1, const Vector3d &p2, const double &y1,
                                       const double &y2, const Vector3d &v1, const double &yd1,
                                       const Vector3d &bbox_min, const Vector3d &bbox_max,
                                       vector<Vector3d> &path);

  // Coarse to fine path searching
  static double searchPath(const Vector3d &p1, const Vector3d &p2, vector<Vector3d> &path);
  static double searchPathBBox(const Vector3d &p1, const Vector3d &p2, const Vector3d &bbox_min,
                               const Vector3d &bbox_max, vector<Vector3d> &path);
  static double searchPathUnknown(const Vector3d &p1, const Vector3d &p2, vector<Vector3d> &path);
  static double searchPathUnknownBBox(const Vector3d &p1, const Vector3d &p2,
                                      const Vector3d &bbox_min, const Vector3d &bbox_max,
                                      vector<Vector3d> &path);

  // Data
  vector<ViewNode::Ptr> neighbors_;
  ViewNode::Ptr parent_;
  Vector3d pos_, vel_;
  double yaw_, yaw_dot_;

  // Parameters shared among nodes
  static double vm_, am_, yd_, ydd_, w_dir_;
  static Astar::Ptr astar_;
  static RayCaster::Ptr caster_;
  static MapServer::Ptr map_server_;
};
} // namespace fast_planner
#endif