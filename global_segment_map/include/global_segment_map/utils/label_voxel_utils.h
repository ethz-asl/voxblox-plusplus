#ifndef GLOBAL_SEGMENT_MAP_UTILS_LABEL_VOXEL_UTILS_H_
#define GLOBAL_SEGMENT_MAP_UTILS_LABEL_VOXEL_UTILS_H_

#include "global_segment_map/label_voxel.h"

#include "voxblox/utils/evaluation_utils.h"
#include "voxblox/utils/voxel_utils.h"

namespace voxblox {

namespace utils {

void updateVoxelLabelAndConfidence(LabelVoxel* label_voxel,
                                   const Label& preferred_label = 0u);

void addVoxelLabelConfidence(const Label& label,
                             const LabelConfidence& confidence,
                             LabelVoxel* label_voxel);

template <>
bool isObservedVoxel(const LabelVoxel& voxel);

}  // namespace utils

template <>
void mergeVoxelAIntoVoxelB(const LabelVoxel& voxel_A, LabelVoxel* voxel_B);

}  // namespace voxblox

#endif  // GLOBAL_SEGMENT_MAP_UTILS_LABEL_VOXEL_UTILS_H_
