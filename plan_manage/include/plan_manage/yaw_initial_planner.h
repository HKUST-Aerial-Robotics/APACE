#ifndef _YAW_INITIAL_PLANNER_H_
#define _YAW_INITIAL_PLANNER_H_

#include "plan_manage/yaw_graph_utils.h"

namespace fast_planner {

class YawInitialPlanner {
public:
  typedef shared_ptr<YawInitialPlanner> Ptr;

  YawInitialPlanner(ros::NodeHandle &nh);
  ~YawInitialPlanner();

  void setMap(const shared_ptr<voxel_mapping::MapServer> &map);
  void searchPathOfYaw(const vector<Eigen::Vector3d> &pos, const vector<Eigen::Vector3d> &acc,
                       const double &dt, vector<double> &path);
  double calcVisibility(const Eigen::Vector3d &pos, const Eigen::Vector3d &acc,
                        const double &yaw_rad, const Eigen::Vector3d &f);
  double calcCoVisibility(YawVertex::Ptr v1, YawVertex::Ptr v2);
  double calcCoVisibility(const Eigen::Vector3d &pos1, const Eigen::Vector3d &pos2,
                          const Eigen::Vector3d &acc1, const Eigen::Vector3d &acc2,
                          const double &yaw_rad1, const double &yaw_rad2,
                          const vector<Eigen::Vector3d> &features);
  void publishYawPath(const vector<Position> &pos, const vector<double> &yaw);

private:
  shared_ptr<MapServer> map_server_;

  // Visualization
  ros::Publisher yaw_path_pub_;
  // params
  double yaw_diff_;
  int half_vert_num_;
  double max_yaw_rate_, w_, w_forwarding_;

  enum WEIGHT_TYPE { NON_UNIFORM, UNIFORM };
  int weight_type_;
};

} // namespace fast_planner

#endif