#include "global_segment_map/label_merge_integrator.h"
#include "global_segment_map/label_voxel.h"

namespace voxblox {
template <>
void mergeVoxelAIntoVoxelB(const LabelVoxel& voxel_A, LabelVoxel* voxel_B) {
  CHECK_NOTNULL(voxel_B);

  if (voxel_A.label == voxel_B->label) {
    voxel_B->label_confidence += voxel_A.label_confidence;
  } else if (voxel_A.label_confidence > voxel_B->label_confidence) {
    voxel_B->label = voxel_A.label;
    voxel_B->label_confidence =
        voxel_A.label_confidence - voxel_B->label_confidence;
  } else {
    voxel_B->label_confidence -= voxel_A.label_confidence;
  }
}
}  // namespace voxblox
