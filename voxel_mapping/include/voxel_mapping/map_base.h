#ifndef MAP_BASE_H
#define MAP_BASE_H

#include "exploration_types.h"
#include "raycast/raycast.h"

#include <Eigen/Eigen>

#include <algorithm>
#include <limits>
#include <memory>

using std::max;
using std::min;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace voxel_mapping {
struct MapConfig {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // map properties
  double resolution_, resolution_inv_;
  Eigen::Vector3d map_size_;
  Eigen::Vector3i map_size_idx_;
  double obstacles_inflation_;
  double local_bound_inflate_, local_map_margin_;
  double default_dist_;
  bool optimistic_, signed_dist_;

  Position map_min_, map_max_;
  Position box_min_, box_max_;
  VoxelIndex box_min_idx_, box_max_idx_;
  Position vbox_min_, vbox_max_;
  VoxelIndex vbox_min_idx_, vbox_max_idx_;
};

template <typename VoxelType> class MapBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef shared_ptr<MapBase<VoxelType>> Ptr;
  typedef shared_ptr<const MapBase<VoxelType>> ConstPtr;

  struct MapData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef shared_ptr<MapData> Ptr;
    typedef shared_ptr<const MapData> ConstPtr;

    Position update_bbox_min_, update_bbox_max_;

    std::vector<VoxelType> data;
  };

  MapBase() : map_data_(new MapData()), raycaster_(new RayCaster()){};
  ~MapBase(){};

  virtual void initMapData();
  virtual void initRaycaster();
  virtual void resetMap();

  virtual void inputPointCloud(const PointCloudType &pointcloud) = 0;

  void positionToIndex(const Position &pos, VoxelIndex &idx);
  VoxelIndex positionToIndex(const Position &pos);
  void indexToPosition(const VoxelIndex &idx, Position &pos);
  Position indexToPosition(const VoxelIndex &idx);
  void positionToAddress(const Position &pos, VoxelAddress &addr);
  VoxelAddress positionToAddress(const Position &pos);
  void indexToAddress(const VoxelIndex &idx, VoxelAddress &addr);
  VoxelAddress indexToAddress(const VoxelIndex &idx);
  bool isInMap(const Position &pos);
  bool isInMap(const VoxelIndex &idx);
  bool isInBox(const Position &pos);
  bool isInBox(const VoxelIndex &idx);
  void boundBox(Position &min, Position &max);
  void boundIndex(VoxelIndex &idx);
  Position closestPointInMap(const Position &point, const Position &sensor_position);

  VoxelType getVoxel(const Position &pos);
  VoxelType getVoxel(const VoxelIndex &idx);
  VoxelType getVoxel(const VoxelAddress &addr);

  void getMapBoundingBox(Position &min, Position &max);
  void getBoxBoundingBox(Position &min, Position &max);

  MapConfig map_config_;

protected:
  typename MapData::Ptr map_data_;
  RayCaster::Ptr raycaster_;
};
} // namespace voxel_mapping

#include "voxel_mapping/map_base_inl.h"

#endif // MAP_BASE_H