#ifndef GLOBAL_SEGMENT_MAP_UTILS_LAYER_UTILS_H_
#define GLOBAL_SEGMENT_MAP_UTILS_LAYER_UTILS_H_

#include <voxblox/utils/layer_utils.h>

#include "global_segment_map/label_voxel.h"

namespace voxblox {
namespace utils {

template <>
inline bool isSameVoxel(const LabelVoxel& voxel_A, const LabelVoxel& voxel_B) {
  bool is_the_same = true;

  is_the_same &= voxel_A.label == voxel_B.label;
  is_the_same &= voxel_A.label_confidence == voxel_B.label_confidence;

  constexpr size_t kNumberOfLabels = 40u;

  for (size_t i = 0u; i < kNumberOfLabels; ++i) {
    is_the_same &= voxel_A.label_count[i].label == voxel_B.label_count[i].label;
    is_the_same &= voxel_A.label_count[i].label_confidence ==
                   voxel_B.label_count[i].label_confidence;
  }

  return is_the_same;
}

}  // namespace utils
}  // namespace voxblox

#endif
