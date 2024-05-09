#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <bspline/bspline_optimizer.h>
#include <bspline/non_uniform_bspline.h>

#include <pathfinding/astar.h>
#include <pathfinding/kinodynamic_astar.h>
#include <pathfinding/topo_prm.h>

#include <plan_env/edt_environment.h>

#include <plan_manage/plan_container.hpp>
#include <plan_manage/yaw_initial_planner.h>

#include <ros/ros.h>

namespace fast_planner {
// Fast Planner Manager
// Key algorithms of mapping and planning are called

enum LOCAL_PLANNER_RESULT { LOCAL_FAIL, LOCAL_SUCCEED };

class FastPlannerManager {
  // SECTION stable
public:
  FastPlannerManager();
  ~FastPlannerManager();

  /* main planning interface */
  void planExploreTraj(const vector<Eigen::Vector3d> &tour, const Eigen::Vector3d &cur_vel,
                       const Eigen::Vector3d &cur_acc, const bool reach_end,
                       const double &time_lb = -1);
  int planLocalMotion(const Vector3d &next_pos, const Vector3d &pos, const Vector3d &vel,
                      const Vector3d &acc, bool &truncated, const double &time_lb);
  void shortenPath(vector<Vector3d> &path);

  void planYaw(const Eigen::Vector3d &start_yaw);
  void planYawPercepAgnostic();
  void planYawCovisibility();
  void planYawPreset(const Eigen::Vector3d &start_yaw, const double &end_yaw);

  void initPlanModules(ros::NodeHandle &nh);
  void setGlobalWaypoints(vector<Eigen::Vector3d> &waypoints);

  bool checkTrajCollision(double &distance);
  void calcNextYaw(const double &last_yaw, double &yaw);

  /* Map save & load service*/
  void saveMapService();
  void loadMapService();

  PlanParameters pp_;
  LocalTrajData local_data_;
  GlobalTrajData global_data_;
  MidPlanData plan_data_;
  EDTEnvironment::Ptr edt_environment_;
  unique_ptr<Astar> path_finder_;
  RayCaster::Ptr caster_;

private:
  string occupancy_map_file_, esdf_map_file_, feature_map_file_;

  /* main planning algorithms & modules */
  voxel_mapping::MapServer::Ptr map_server_;
  vector<BsplineOptimizer::Ptr> bspline_optimizers_;

  void updateTrajInfo();

  // topology guided optimization

  Eigen::MatrixXd paramLocalTraj(double start_t, double &dt, double &duration);
  Eigen::MatrixXd reparamLocalTraj(const double &start_t, const double &duration, const double &dt);

  void selectBestTraj(NonUniformBspline &traj);
  void refineTraj(NonUniformBspline &best_traj);
  void reparamBspline(NonUniformBspline &bspline, double ratio, Eigen::MatrixXd &ctrl_pts,
                      double &dt, double &time_inc);

public:
  typedef shared_ptr<FastPlannerManager> Ptr;

  void planYawActMap(const Eigen::Vector3d &start_yaw);
  void test();
  void searchFrontier(const Eigen::Vector3d &p);

private:
  // unique_ptr<FrontierFinder> frontier_finder_;
  // unique_ptr<HeadingPlanner> heading_planner_;
  unique_ptr<YawInitialPlanner> yaw_initial_planner_;
  unique_ptr<VisibilityUtil> visib_util_;

  double max_yaw_vel = 0.0;

  // Benchmark method, local exploration
public:
  bool localExplore(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                    Eigen::Vector3d end_pt);

  // !SECTION
};
} // namespace fast_planner

#endif