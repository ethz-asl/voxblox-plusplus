#ifndef GLOBAL_SEGMENT_MAP_LABEL_BLOCK_H_
#define GLOBAL_SEGMENT_MAP_LABEL_BLOCK_H_

#include <voxblox/core/block.h>
#include <voxblox/core/layer.h>
#include <voxblox/integrator/merge_integration.h>

#include "global_segment_map/common.h"
#include "global_segment_map/label_voxel.h"

namespace voxblox {

template <>
void Block<LabelVoxel>::mergeBlock(const Block<LabelVoxel>& other_block);

template <>
void Block<LabelVoxel>::deserializeFromIntegers(
    const std::vector<uint32_t>& data);

template <>
void Block<LabelVoxel>::serializeToIntegers(std::vector<uint32_t>* data) const;

}  // namespace voxblox

#endif  // GLOBAL_SEGMENT_MAP_LABEL_BLOCK_H_
