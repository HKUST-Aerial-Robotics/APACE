#ifndef _HEADING_PLANNER_H_
#define _HEADING_PLANNER_H_

#include "active_perception/yaw_graph_utils.h"
#include "raycast/raycast.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

class RayCaster;

namespace fast_planner {

class CastFlags {
private:
  /* data */
  vector<char> flags_;
  Eigen::Vector3i lb_, ub_, cells_;

public:
  CastFlags() {
  }
  CastFlags(const int& size) {
    flags_.resize(size);
  }
  ~CastFlags() {
  }

  void reset(const Eigen::Vector3i& lb, const Eigen::Vector3i& ub) {
    lb_ = lb;
    ub_ = ub;
    cells_ = ub_ - lb_;
    fill(flags_.begin(), flags_.end(), 0);
  }

  inline int address(const Eigen::Vector3i& idx) {
    Eigen::Vector3i diff = idx - lb_;
    return diff[2] + diff[1] * cells_[2] + diff[0] * cells_[1] * cells_[2];
  }

  inline char getFlag(const Eigen::Vector3i& idx) {
    return flags_[address(idx)];
  }

  inline void setFlag(const Eigen::Vector3i& idx, const char& f) {
    flags_[address(idx)] = f;
  }
};

class HeadingPlanner {
public:
  HeadingPlanner(ros::NodeHandle& nh);
  ~HeadingPlanner();

  void setMap(const shared_ptr<voxel_mapping::MapServer>& map);
  void searchPathOfYaw(const vector<Eigen::Vector3d>& pts, const vector<double>& yaws, const double& dt,
                       const Eigen::MatrixXd& ctrl_pts, vector<double>& path);

  // frontier-based IG, not good to evaluate information gain
  void setFrontier(const vector<vector<Eigen::Vector3d>>& frontier);
  void calcVisibFrontier(const Eigen::Vector3d& pt, const double& yaw,
                         unordered_map<int, int>& visib_idx);
  void showVisibFrontier(const vector<YawVertex::Ptr>& path);
  double calcInfoGain(const Eigen::Vector3d& pt, const double& yaw, const int& task_id);

private:
  void setTransform(const Eigen::Matrix3d& R_wb, const Eigen::Vector3d& t_wb);
  bool insideFoV(const Eigen::Vector4d& pw);

  // iterate within volume and check visibility, voexl are weighted by distance
  double calcInformationGain(const Eigen::Vector3d& pt, const double& yaw,
                             const Eigen::MatrixXd& ctrl_pts, const int& task_id);
  // iterate within volume and check visibility, voexl are weighted uniformly
  bool insideFoV(const Eigen::Vector3d& pw, const Eigen::Vector3d& pc,
                 const vector<Eigen::Vector3d>& normals);
  void distToPathAndCurPos(const Eigen::Vector3d& check_pt, const Eigen::MatrixXd& ctrl_pts,
                           std::pair<double, double>& dists, bool debug = false);
  void axisAlignedBoundingBox(const vector<Eigen::Vector3d>& points, Eigen::Vector3d& lb,
                              Eigen::Vector3d& ub);
  void visualizeBox(const Eigen::Vector3d& lb, const Eigen::Vector3d& ub);
  void calcFovAABB(const Eigen::Matrix3d& R_wc, const Eigen::Vector3d& t_wc, Eigen::Vector3i& lb,
                   Eigen::Vector3i& ub);
  void initCastFlag(const Eigen::Vector3d& pos);

  pcl::PointCloud<pcl::PointXYZ>::Ptr frontier_;
  pcl::KdTreeFLANN<pcl::PointXYZ> ft_kdtree_;

  shared_ptr<MapServer> map_server_;
  vector<unique_ptr<RayCaster>> casters_;

  // normals of camera FoV seperating hyperplane
  Eigen::Vector3d n_top_, n_bottom_, n_left_, n_right_;
  // vertices of FoV
  Eigen::Vector3d lefttop_, righttop_, leftbottom_, rightbottom_;
  // flags for accelerated raycasting, 0: unvisited, 1: visible, 2: invisible
  CastFlags cast_flags_;
  // camera parameters
  double tanyz_, tanxz_, near_, far_;
  Eigen::Matrix4d T_cb_, T_bc_;  // transform between camera and body frame
  // debug
  ros::Publisher frontier_pub_, visib_pub_, box_pub_;
  // params
  double yaw_diff_, lambda1_, lambda2_;
  int half_vert_num_;
  double max_yaw_rate_, w_;

  enum WEIGHT_TYPE { NON_UNIFORM, UNIFORM };
  int weight_type_;
};

}  // namespace fast_planner

#endif