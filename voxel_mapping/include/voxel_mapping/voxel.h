#ifndef VOXEL_H
#define VOXEL_H

#include "exploration_types.h"

namespace voxel_mapping {

enum class OccupancyType {
  UNKNOWN,
  UNKNOWN_FRONTIER_OCCUPIED,
  UNKNOWN_FRONTIER_FREE,
  OCCUPIED,
  FREE
};

struct OccupancyVoxel {
  OccupancyType value = OccupancyType::UNKNOWN;
};

struct TSDFVoxel {
  FloatingPoint value = 0.0, weight = 0.0;
};

struct ESDFVoxel {
  FloatingPoint value = 0.0;
};

} // namespace voxel_mapping

#endif // VOXEL_H