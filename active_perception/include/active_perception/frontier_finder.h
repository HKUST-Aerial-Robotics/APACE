#ifndef _FRONTIER_FINDER_H_
#define _FRONTIER_FINDER_H_

#include "plan_env/edt_environment.h"
#include "raycast/raycast.h"

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

#include <list>
#include <memory>
#include <utility>
#include <vector>

using Eigen::Vector3d;
using std::list;
using std::pair;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

class RayCaster;

namespace fast_planner {
class EDTEnvironment;
class PerceptionUtils;

// Viewpoint to cover a frontier cluster
struct Viewpoint {
  // Position and heading
  Vector3d pos_;
  double yaw_;
  // Fraction of the cluster that can be covered
  // double fraction_;
  int visib_num_;
};

// A frontier cluster, the viewpoints to cover it
struct Frontier {
  // TODO: Unique ID of the frontier: timestamp + average position + size
  unsigned int unqiue_id_;
  // Idx of cluster
  int id_;

  // Complete voxels belonging to the cluster
  vector<Vector3d> cells_;
  // down-sampled voxels filtered by voxel grid filter
  vector<Vector3d> filtered_cells_;
  // Average position of all voxels
  Vector3d average_;
  // Viewpoints that can cover the cluster
  vector<Viewpoint> viewpoints_;
  // Bounding box of cluster, center & 1/2 side length
  Vector3d box_min_, box_max_;
  // Path and cost from this cluster to other clusters
  list<vector<Vector3d>> paths_;
  list<double> costs_;

  Eigen::Vector2d first_pc_;
};

class FrontierFinder {
public:
  FrontierFinder(const shared_ptr<EDTEnvironment> &edt, ros::NodeHandle &nh);
  ~FrontierFinder();

  void searchFrontiers();
  void computeFrontiersToVisit();

  void getFrontiers(vector<vector<Vector3d>> &clusters);
  void getDormantFrontiers(vector<vector<Vector3d>> &clusters);
  void getFrontierBoxes(vector<pair<Vector3d, Vector3d>> &boxes);
  // Get viewpoint with highest coverage for each frontier
  void getTopViewpointsInfo(const Vector3d &cur_pos, vector<Vector3d> &points, vector<double> &yaws,
                            vector<Vector3d> &averages);
  // Get several viewpoints for a subset of frontiers
  void getViewpointsInfo(const Vector3d &cur_pos, const vector<int> &ids, const int &view_num,
                         const double &max_decay, vector<vector<Vector3d>> &points,
                         vector<vector<double>> &yaws);
  void updateFrontierCostMatrix();
  void getFullCostMatrix(const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d cur_yaw,
                         Eigen::MatrixXd &mat);
  void getSwarmCostMatrix(const vector<Vector3d> &positions, const vector<Vector3d> &velocities,
                          const vector<double> yaws, Eigen::MatrixXd &mat);
  void getSwarmCostMatrix(const vector<Vector3d> &positions, const vector<Vector3d> &velocities,
                          const vector<double> &yaws, const vector<int> &ftr_ids,
                          const vector<Eigen::Vector3d> &grid_pos, Eigen::MatrixXd &mat);
  void getSingleCellCostMatrix(const Vector3d &cur_pos, const Vector3d &cur_vel,
                               const Vector3d &cur_yaw, const vector<int> &ftr_ids,
                               const Vector3d &next_grid_pos, Eigen::MatrixXd &mat);
  void getPathForTour(const Vector3d &pos, const vector<int> &frontier_ids, vector<Vector3d> &path);
  void getFrontierFirstPCs(vector<Eigen::Vector2d> &pcs);

  void setNextFrontier(const int &id);
  bool isFrontierCovered();
  void wrapYaw(double &yaw);

  shared_ptr<PerceptionUtils> percep_utils_;

private:
  void splitLargeFrontiers(list<Frontier> &frontiers);
  bool splitHorizontally(Frontier &frontier, list<Frontier> &splits);
  void mergeFrontiers(Frontier &ftr1, const Frontier &ftr2);
  bool isFrontierChanged(const Frontier &ft);
  bool haveOverlap(const Vector3d &min1, const Vector3d &max1, const Vector3d &min2,
                   const Vector3d &max2);
  void computeFrontierInfo(Frontier &frontier);
  void downsample(const vector<Vector3d> &cluster_in, vector<Vector3d> &cluster_out);
  void sampleViewpoints(Frontier &frontier);

  bool isNearUnknown(const Vector3d &pos);
  bool isNearOccupied(const Vector3d &pos);
  int countVisibleCells(const Vector3d &pos, const double &yaw, const vector<Vector3d> &cluster);
  vector<Eigen::Vector3i> sixNeighbors(const Eigen::Vector3i &voxel);
  vector<Eigen::Vector3i> tenNeighbors(const Eigen::Vector3i &voxel);
  vector<Eigen::Vector3i> allNeighbors(const Eigen::Vector3i &voxel);
  bool isNeighborUnknown(const Eigen::Vector3i &voxel);
  void expandFrontier(const Eigen::Vector3i &first /* , const int& depth, const int& parent_id */);

  // Wrapper of sdf map
  int toadr(const Eigen::Vector3i &idx);
  bool knownfree(const Eigen::Vector3i &idx);
  bool inmap(const Eigen::Vector3i &idx);

  // Deprecated
  Eigen::Vector3i searchClearVoxel(const Eigen::Vector3i &pt);
  bool isInBoxes(const vector<pair<Vector3d, Vector3d>> &boxes, const Eigen::Vector3i &idx);
  bool canBeMerged(const Frontier &ftr1, const Frontier &ftr2);
  void findViewpoints(const Vector3d &sample, const Vector3d &ftr_avg, vector<Viewpoint> &vps);

  // Data
  vector<char> frontier_flag_;
  list<Frontier> frontiers_, dormant_frontiers_, tmp_frontiers_;
  vector<int> removed_ids_;
  list<Frontier>::iterator first_new_ftr_;
  Frontier next_frontier_;

  // Params
  int cluster_min_;
  double cluster_size_xy_, cluster_size_z_;
  double candidate_rmax_, candidate_rmin_, candidate_dphi_, min_candidate_dist_,
      min_candidate_clearance_, min_candidate_occupied_clearance_;
  int down_sample_;
  double min_view_finish_fraction_, resolution_;
  int min_visib_num_, candidate_rnum_;
  double ground_height_, ground_offset_;

  // Utils
  shared_ptr<EDTEnvironment> edt_env_;
  unique_ptr<RayCaster> raycaster_;
};

} // namespace fast_planner
#endif