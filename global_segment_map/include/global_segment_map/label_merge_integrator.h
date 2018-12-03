#ifndef GLOBAL_SEGMENT_MAP_LABEL_MERGE_INTEGRATOR_H_
#define GLOBAL_SEGMENT_MAP_LABEL_MERGE_INTEGRATOR_H_

#include <voxblox/integrator/merge_integration.h>

#include "global_segment_map/label_voxel.h"

namespace voxblox {
template <>
void mergeVoxelAIntoVoxelB(const LabelVoxel& voxel_A, LabelVoxel* voxel_B);
}  // namespace voxblox

#endif  // GLOBAL_SEGMENT_MAP_LABEL_MERGE_INTEGRATOR_H_
