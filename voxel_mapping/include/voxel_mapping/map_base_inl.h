#include "voxel_mapping/map_base.h"

namespace voxel_mapping {
template <typename VoxelType> void MapBase<VoxelType>::initMapData() {
  map_data_->data.resize(map_config_.map_size_idx_.prod());
}

template <typename VoxelType> void MapBase<VoxelType>::initRaycaster() {
  raycaster_->setParams(map_config_.resolution_, map_config_.map_min_);
}
template <typename VoxelType> void MapBase<VoxelType>::resetMap() {
  map_data_->data.clear();
  initMapData();
}

template <typename VoxelType>
void MapBase<VoxelType>::positionToIndex(const Position &pos, VoxelIndex &idx) {
  for (int i = 0; i < 3; ++i)
    idx(i) = floor((pos(i) - map_config_.map_min_(i)) * map_config_.resolution_inv_);
}

template <typename VoxelType> VoxelIndex MapBase<VoxelType>::positionToIndex(const Position &pos) {
  VoxelIndex idx;
  positionToIndex(pos, idx);
  return idx;
}

template <typename VoxelType>
void MapBase<VoxelType>::indexToPosition(const VoxelIndex &idx, Position &pos) {
  for (int i = 0; i < 3; ++i)
    pos(i) = (idx(i) + 0.5) * map_config_.resolution_ + map_config_.map_min_(i);
}

template <typename VoxelType> Position MapBase<VoxelType>::indexToPosition(const VoxelIndex &idx) {
  Position pos;
  indexToPosition(idx, pos);
  return pos;
}

template <typename VoxelType>
void MapBase<VoxelType>::positionToAddress(const Position &pos, VoxelAddress &addr) {
  addr = indexToAddress(positionToIndex(pos));
}

template <typename VoxelType>
VoxelAddress MapBase<VoxelType>::positionToAddress(const Position &pos) {
  VoxelIndex idx;
  positionToIndex(pos, idx);
  return indexToAddress(idx);
}

template <typename VoxelType>
void MapBase<VoxelType>::indexToAddress(const VoxelIndex &idx, VoxelAddress &addr) {
  addr = idx[0] * map_config_.map_size_idx_(1) * map_config_.map_size_idx_(2) +
         idx[1] * map_config_.map_size_idx_(2) + idx[2];
}

template <typename VoxelType>
VoxelAddress MapBase<VoxelType>::indexToAddress(const VoxelIndex &idx) {
  return idx[0] * map_config_.map_size_idx_(1) * map_config_.map_size_idx_(2) +
         idx[1] * map_config_.map_size_idx_(2) + idx[2];
}

template <typename VoxelType> bool MapBase<VoxelType>::isInMap(const Position &pos) {
  if (pos(0) < map_config_.map_min_(0) + 1e-4 || pos(1) < map_config_.map_min_(1) + 1e-4 ||
      pos(2) < map_config_.map_min_(2) + 1e-4)
    return false;
  if (pos(0) > map_config_.map_max_(0) - 1e-4 || pos(1) > map_config_.map_max_(1) - 1e-4 ||
      pos(2) > map_config_.map_max_(2) - 1e-4)
    return false;
  return true;
}

template <typename VoxelType> bool MapBase<VoxelType>::isInMap(const VoxelIndex &idx) {
  if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0)
    return false;
  if (idx(0) > map_config_.map_size_idx_(0) - 1 || idx(1) > map_config_.map_size_idx_(1) - 1 ||
      idx(2) > map_config_.map_size_idx_(2) - 1)
    return false;
  return true;
}

template <typename VoxelType> bool MapBase<VoxelType>::isInBox(const Position &pos) {
  for (int i = 0; i < 3; ++i) {
    if (pos[i] < map_config_.box_min_[i] || pos[i] > map_config_.box_max_[i]) {
      return false;
    }
  }
  return true;
}

template <typename VoxelType> bool MapBase<VoxelType>::isInBox(const VoxelIndex &idx) {
  for (int i = 0; i < 3; ++i) {
    if (idx[i] < map_config_.box_min_idx_[i] || idx[i] > map_config_.box_max_idx_[i]) {
      return false;
    }
  }
  return true;
}

template <typename VoxelType>
void MapBase<VoxelType>::boundBox(Position &min_pos, Position &max_pos) {
  for (int i = 0; i < 3; ++i) {
    min_pos[i] = max(min_pos[i], map_config_.box_min_[i]);
    max_pos[i] = min(max_pos[i], map_config_.box_max_[i]);
  }
}

template <typename VoxelType> void MapBase<VoxelType>::boundIndex(VoxelIndex &idx) {
  // Bound index to [0, map_size_idx_), i.e. inside map
  idx(0) = max(min(idx(0), map_config_.map_size_idx_(0) - 1), 0);
  idx(1) = max(min(idx(1), map_config_.map_size_idx_(1) - 1), 0);
  idx(2) = max(min(idx(2), map_config_.map_size_idx_(2) - 1), 0);
}

template <typename VoxelType>
Position MapBase<VoxelType>::closestPointInMap(const Position &point,
                                               const Position &sensor_position) {
  Position diff = point - sensor_position;
  Position max_tc = map_config_.map_max_ - sensor_position;
  Position min_tc = map_config_.map_min_ - sensor_position;

  FloatingPoint min_t = std::numeric_limits<FloatingPoint>::max();
  for (int i = 0; i < 3; ++i) {
    if (fabs(diff[i]) > 0) {
      FloatingPoint t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t)
        min_t = t1;
      FloatingPoint t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }

  return sensor_position + (min_t - 1e-3) * diff;
}

template <typename VoxelType> VoxelType MapBase<VoxelType>::getVoxel(const Position &pos) {
  return getVoxel(positionToAddress(pos));
}

template <typename VoxelType> VoxelType MapBase<VoxelType>::getVoxel(const VoxelIndex &idx) {
  return getVoxel(indexToAddress(idx));
}

template <typename VoxelType> VoxelType MapBase<VoxelType>::getVoxel(const VoxelAddress &addr) {
  return map_data_->data[addr];
}

template <typename VoxelType> void MapBase<VoxelType>::getMapBoundingBox(Position &min, Position &max) {
  min = map_config_.map_min_;
  max = map_config_.map_max_;
}

template <typename VoxelType> void MapBase<VoxelType>::getBoxBoundingBox(Position &min, Position &max) {
  min = map_config_.box_min_;
  max = map_config_.box_max_;
}

} // namespace voxel_mapping