#ifndef TSDF_H
#define TSDF_H

#include "voxel_mapping/map_base.h"
#include "voxel_mapping/voxel.h"

namespace voxel_mapping {
class ESDF;
class OccupancyGrid;

class TSDF : public MapBase<TSDFVoxel> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef shared_ptr<TSDF> Ptr;
  typedef shared_ptr<const TSDF> ConstPtr;

  struct Config {
    enum class DepthAxis { X, Y, Z };
    FloatingPoint truncated_dist_, truncated_dist_behind_;
    FloatingPoint raycast_min_, raycast_max_;
    FloatingPoint result_truncated_dist_;
    FloatingPoint epsilon_ = 1e-4;
    DepthAxis depth_axis_;
  };

  TSDF() : raycast_id_(0){};
  ~TSDF(){};

  void inputPointCloud(const PointCloudType &pointcloud);

  FloatingPoint computeDistance(const Position &origin, const Position &point,
                                const Position &voxel);
  FloatingPoint computeWeight(const FloatingPoint &sdf, const FloatingPoint &depth);
  void updateTSDFVoxel(const VoxelAddress &addr, const FloatingPoint &sdf,
                       const FloatingPoint &weight = 1.0);

  void setESDF(const shared_ptr<ESDF> &esdf) { esdf_ = esdf; }

  void setOccupancyGrid(const shared_ptr<OccupancyGrid> &occupancy_grid) {
    occupancy_grid_ = occupancy_grid;
  }

  void getUpdatedBox(Eigen::Vector3d &bmin, Eigen::Vector3d &bmax, bool reset = false);

  Config config_;

private:
  bool reset_updated_bbox_;
  int raycast_id_;

  shared_ptr<ESDF> esdf_;
  shared_ptr<OccupancyGrid> occupancy_grid_;
};
} // namespace voxel_mapping

#endif // TSDF_H