#ifndef ESDF_H
#define ESDF_H

#include "voxel_mapping/map_base.h"
#include "voxel_mapping/voxel.h"

namespace voxel_mapping {
class TSDF;
class OccupancyGrid;

class ESDF : public MapBase<ESDFVoxel> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef shared_ptr<ESDF> Ptr;
  typedef shared_ptr<const ESDF> ConstPtr;

  struct Config {
    FloatingPoint epsilon_ = 1e-4;
  };

  ESDF(){};
  ~ESDF(){};

  void initMapData() override {
    map_data_->data.resize(map_config_.map_size_idx_.prod());
    tmp_buffer1_.resize(map_config_.map_size_idx_.prod());
    tmp_buffer2_.resize(map_config_.map_size_idx_.prod());
  };

  void inputPointCloud(const PointCloudType &pointcloud);
  void updateLocalESDF(const VoxelIndex &idx_min, const VoxelIndex &idx_max);

  template <typename F_get_val, typename F_set_val>
  void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);
  double getDistance(const Position &pos);
  double getDistance(const VoxelIndex &idx);
  void getDistanceAndGradient(const Position &pos, double &distance, Eigen::Vector3d &gradient);
  void getDistanceAndGradient(const VoxelIndex &idx, double &distance, Eigen::Vector3d &gradient);

  void setTSDF(const shared_ptr<TSDF> &tsdf) { tsdf_ = tsdf; }
  void loadMap(const std::string &filename);

  void setOccupancyGrid(const shared_ptr<OccupancyGrid> &occupancy_grid) {
    occupancy_grid_ = occupancy_grid;
  }

  Config config_;

private:
  shared_ptr<TSDF> tsdf_;
  shared_ptr<OccupancyGrid> occupancy_grid_;

  std::vector<ESDFVoxel> tmp_buffer1_, tmp_buffer2_;
};
} // namespace voxel_mapping

#endif // ESDF_H