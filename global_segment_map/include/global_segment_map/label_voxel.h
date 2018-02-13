#ifndef GLOBAL_SEGMENT_MAP_LABEL_VOXEL_H_
#define GLOBAL_SEGMENT_MAP_LABEL_VOXEL_H_

#include <cstdint>

#include <voxblox/core/color.h>
#include <voxblox/core/common.h>
#include <voxblox/core/voxel.h>

namespace voxblox {

struct LabelCount {
  Label label = 0u;
  LabelConfidence label_confidence = 0.0f;
};

struct LabelVoxel {
  Label label = 0u;
  LabelConfidence label_confidence = 0u;
  LabelCount label_count[20];
};

namespace voxel_types {
const std::string kLabel = "label";
}  // namespace voxel_types

template <>
inline std::string getVoxelType<LabelVoxel>() {
  return voxel_types::kLabel;
}

}  // namespace voxblox

#endif  // GLOBAL_SEGMENT_MAP_LABEL_VOXEL_H_
