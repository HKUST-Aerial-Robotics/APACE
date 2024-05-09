#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H

#include <fstream>
#include <iostream>
#include <string>

#include "voxel_mapping/map_base.h"
#include "voxel_mapping/voxel.h"

using std::string;

namespace voxel_mapping {
class TSDF;

class OccupancyGrid : public MapBase<OccupancyVoxel> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef shared_ptr<OccupancyGrid> Ptr;
  typedef shared_ptr<const OccupancyGrid> ConstPtr;

  struct Config {
    enum class MODE { GEN_TSDF, GEN_PCL };

    double TSDF_cutoff_dist_;
    MODE mode_;
  };

  OccupancyGrid(){};
  ~OccupancyGrid(){};

  void inputPointCloud(const PointCloudType &pointcloud);

  void updateOccupancyVoxel(const VoxelAddress &addr);

  void setTSDF(const shared_ptr<TSDF> &tsdf) { tsdf_ = tsdf; }

  bool queryOcclusion(const Position &sensor_position, const Position &feature_point,
                      const double raycast_tolerance);

  OccupancyType queryOccupancy(const Position &pos);

  void publishPointcloud(PointCloudType pcd);

  void saveMap(const string &filename);
  void loadMap(const string &filename);
  void loadMapFromPcd(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

  Config config_;

private:
  shared_ptr<TSDF> tsdf_;
};
} // namespace voxel_mapping

#endif // ESDF_H