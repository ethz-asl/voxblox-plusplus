#ifndef GLOBAL_SEGMENT_MAP_LAYER_EVALUATION_H_
#define GLOBAL_SEGMENT_MAP_LAYER_EVALUATION_H_

#include <vector>

#include <voxblox/core/common.h>

namespace voxblox {
template <typename VoxelType>
void evaluateLayerAtPoses(
    const utils::VoxelEvaluationMode& voxel_evaluation_mode,
    const Layer<VoxelType>& tsdf_layer,
    const std::shared_ptr<Layer<VoxelType>>& merged_object_layer_O,
    const std::vector<Transformation>& transforms_W_O,
    std::vector<utils::VoxelEvaluationDetails>*
        voxel_evaluation_details_vector);
}

#endif  // GLOBAL_SEGMENT_MAP_LAYER_EVALUATION_H_

#include "global_segment_map/layer_evaluation_inl.h"
