#ifndef GLOBAL_SEGMENT_MAP_LABEL_VOXEL_H_
#define GLOBAL_SEGMENT_MAP_LABEL_VOXEL_H_

#include <cstdint>
#include <string>

#include <voxblox/core/voxel.h>
#include <voxblox/utils/layer_utils.h>

#include "global_segment_map/common.h"

namespace voxblox {

struct LabelVoxel {
  Label label = 0u;
  LabelConfidence label_confidence = 0u;
  LabelCount label_count[3];
};

namespace voxel_types {
const std::string kLabel = "label";
}  // namespace voxel_types

template <>
inline std::string getVoxelType<LabelVoxel>() {
  return voxel_types::kLabel;
}

namespace utils {
template <>
inline bool isSameVoxel(const LabelVoxel& voxel_A, const LabelVoxel& voxel_B) {
  bool is_the_same = true;

  is_the_same &= voxel_A.label == voxel_B.label;
  is_the_same &= voxel_A.label_confidence == voxel_B.label_confidence;

  for (int i = 0; i < 40; ++i) {
    is_the_same &= voxel_A.label_count[i].label == voxel_B.label_count[i].label;
    is_the_same &= voxel_A.label_count[i].label_confidence ==
                   voxel_B.label_count[i].label_confidence;
  }

  return is_the_same;
}
}  // namespace utils
}  // namespace voxblox

#endif  // GLOBAL_SEGMENT_MAP_LABEL_VOXEL_H_
