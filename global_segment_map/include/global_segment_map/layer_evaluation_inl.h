#ifndef GLOBAL_SEGMENT_MAP_LAYER_EVALUATION_INL_H_
#define GLOBAL_SEGMENT_MAP_LAYER_EVALUATION_INL_H_

#include <vector>

#include <voxblox/core/common.h>
#include <voxblox/integrator/merge_integration.h>

namespace voxblox {
template <typename VoxelType>
void evaluateLayerAtPoses(
    const utils::VoxelEvaluationMode& voxel_evaluation_mode,
    const Layer<VoxelType>& layer,
    const std::shared_ptr<Layer<VoxelType>>& merged_object_layer_O,
    const std::vector<Transformation>& transforms_W_O,
    std::vector<utils::VoxelEvaluationDetails>*
        voxel_evaluation_details_vector) {
  CHECK_NOTNULL(voxel_evaluation_details_vector);
  size_t idx = 0u;
  // Check if world TSDF layer agrees with merged object at all object poses.
  for (const Transformation& transform_W_O : transforms_W_O) {
    std::shared_ptr<Layer<VoxelType>> merged_object_layer_W;

    // Transform merged object into the world frame.
    transformLayer<VoxelType>(*merged_object_layer_O.get(),
                                       transform_W_O.inverse(),
                                       merged_object_layer_W.get());

    utils::VoxelEvaluationDetails voxel_evaluation_details;
    // Evaluate the RMSE of the merged object layer in the world layer.
    utils::evaluateLayersRmse(
        layer, *merged_object_layer_W,
        voxel_evaluation_mode, &voxel_evaluation_details);
    voxel_evaluation_details_vector->push_back(voxel_evaluation_details);
    ++idx;
    CHECK_LT(idx, transforms_W_O.size());
  }
}
}  // namespace voxblox
#endif  // GLOBAL_SEGMENT_MAP_LAYER_EVALUATION_INL_H_
