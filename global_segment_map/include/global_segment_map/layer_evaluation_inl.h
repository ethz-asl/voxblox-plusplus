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
    const Layer<VoxelType>& merged_object_layer_O,
    const std::vector<Transformation>& transforms_W_O,
    std::vector<utils::VoxelEvaluationDetails>*
        voxel_evaluation_details_vector) {
  CHECK_NOTNULL(voxel_evaluation_details_vector);
  // Check if world TSDF layer agrees with merged object at all object poses.
  for (const Transformation& transform_W_O : transforms_W_O) {
    Layer<VoxelType> merged_object_layer_W(
        merged_object_layer_O.voxel_size(),
        merged_object_layer_O.voxels_per_side());

    // Transform merged object into the world frame.
    transformLayer<VoxelType>(merged_object_layer_O, transform_W_O,
                              &merged_object_layer_W);

    utils::VoxelEvaluationDetails voxel_evaluation_details;
    // Evaluate the RMSE of the merged object layer in the world layer.
    utils::evaluateLayersRmse(layer, merged_object_layer_W,
                              voxel_evaluation_mode, &voxel_evaluation_details);
    voxel_evaluation_details_vector->push_back(voxel_evaluation_details);
  }
}

template <typename VoxelType>
void evaluateLayerAtPoses(
    const utils::VoxelEvaluationMode& voxel_evaluation_mode,
    const Layer<VoxelType>& layer,
    const Layer<VoxelType>& merged_object_layer_O,
    const modelify::TransformationVector& transforms_W_O,
    std::vector<utils::VoxelEvaluationDetails>*
        voxel_evaluation_details_vector) {
  CHECK_NOTNULL(voxel_evaluation_details_vector);
  std::vector<Transformation> kindr_transforms_W_O;
  for (const modelify::Transformation& transform_W_O : transforms_W_O) {
    kindr_transforms_W_O.emplace_back(transform_W_O);
  }
  evaluateLayerAtPoses(voxel_evaluation_mode, layer, merged_object_layer_O,
                       kindr_transforms_W_O, voxel_evaluation_details_vector);
}

}  // namespace voxblox
#endif  // GLOBAL_SEGMENT_MAP_LAYER_EVALUATION_INL_H_
