#include "voxel_mapping/occupancy_grid.h"
#include "voxel_mapping/tsdf.h"

namespace voxel_mapping {
void OccupancyGrid::inputPointCloud(const PointCloudType &pointcloud) {}

void OccupancyGrid::updateOccupancyVoxel(const VoxelAddress &addr) {
  if (tsdf_->getVoxel(addr).weight < 1e-5) {
    map_data_->data[addr].value = OccupancyType::UNKNOWN;
    return;
  }

  // if (tsdf_->getVoxel(addr).weight < mp_->tsdf_observation_th_) {
  //   if (tsdf_->getVoxel(addr).value < mp_->tsdf_occu_th_) {
  //     map_data_->data[addr].value = OccupancyType::UNKNOWN_FRONTIER_OCCUPIED;
  //   } else {
  //     map_data_->data[addr].value = OccupancyType::UNKNOWN_FRONTIER_FREE;
  //   }
  //   return;
  // }

  if (tsdf_->getVoxel(addr).value < config_.TSDF_cutoff_dist_) {
    map_data_->data[addr].value = OccupancyType::OCCUPIED;
  } else {
    map_data_->data[addr].value = OccupancyType::FREE;
  }
}

bool OccupancyGrid::queryOcclusion(const Position &sensor_position, const Position &feature_point,
                                   const double raycast_tolerance) {
  VoxelIndex voxel_idx;
  VoxelAddress voxel_addr;
  Position voxel_pos;
  Position raycast_start, raycast_end;
  Position unit_dir = (feature_point - sensor_position).normalized();
  double truncated_length = (feature_point - sensor_position).norm() - raycast_tolerance;
  raycast_start = sensor_position;
  raycast_end = sensor_position + unit_dir * (truncated_length > 0. ? truncated_length : 0.);
  raycaster_->input(raycast_start, raycast_end);
  while (raycaster_->nextId(voxel_idx)) {
    voxel_addr = indexToAddress(voxel_idx);
    voxel_pos = indexToPosition(voxel_idx);
    if (map_data_->data[voxel_addr].value == OccupancyType::OCCUPIED) {
      return true;
    }
  }
  return false;
}

OccupancyType OccupancyGrid::queryOccupancy(const Position &pos){
  VoxelAddress add = positionToAddress(pos);
  return map_data_->data[add].value;
}

void OccupancyGrid::saveMap(const string &filename) {
  std::ofstream ofs(filename);
  if (!ofs.is_open()) {
    std::cerr << "Failed to open file: " << filename << std::endl;
    return;
  }

  for (size_t i = 0; i < map_data_->data.size(); ++i) {
    ofs << (int)map_data_->data[i].value << " ";
  }

  ofs.close();
}

void OccupancyGrid::loadMap(const string &filename) {
  std::ifstream ifs(filename);
  if (!ifs.is_open()) {
    std::cerr << "Failed to open file: " << filename << std::endl;
    return;
  }
  for (size_t i = 0; i < map_data_->data.size(); ++i) {
    int value;
    ifs >> value;
    map_data_->data[i].value = (OccupancyType)value;
  }
  ifs.close();
}

void OccupancyGrid::loadMapFromPcd(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
  for (pcl::PointXYZ p : cloud->points) {
    Position point(p.x, p.y, p.z);
    VoxelIndex voxel_idx = positionToIndex(point);
    VoxelAddress voxel_addr = indexToAddress(voxel_idx);
    map_data_->data[voxel_addr].value = OccupancyType::OCCUPIED;
  }
}

} // namespace voxel_mapping