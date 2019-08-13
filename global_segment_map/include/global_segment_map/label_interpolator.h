#ifndef GLOBAL_SEGMENT_MAP_LABEL_INTERPOLATOR_H_
#define GLOBAL_SEGMENT_MAP_LABEL_INTERPOLATOR_H_

#include "global_segment_map/utils/label_voxel_utils.h"

namespace voxblox {

template <>
inline LabelVoxel Interpolator<LabelVoxel>::interpVoxel(
    const InterpVector& q_vector, const LabelVoxel** voxels) {
  LabelVoxel voxel;

  // TODO(margaritaG): Right now all voxels in the layer being interpolated can
  // only belong to the same Label. Therefore, it is relatively to always use
  // the same label, rather than interpolating weights. However, to properly
  // track weights and be robust to different scenarios, need to implement
  // trilinear interpolation here.
  std::copy(std::begin(voxels[0]->label_count),
            std::end(voxels[0]->label_count), std::begin(voxel.label_count));

  utils::updateVoxelLabelAndConfidence(&voxel);

  return voxel;
}
}  // namespace voxblox

#endif  // GLOBAL_SEGMENT_MAP_LABEL_INTERPOLATOR_H_
