#ifndef GLOBAL_FEATURE_MAP_FEATURE_BLOCK_INL_H_
#define GLOBAL_FEATURE_MAP_FEATURE_BLOCK_INL_H_

#include <vector>

#include <voxblox/core/common.h>

namespace voxblox {

template <typename FeatureType>
void FeatureBlock<FeatureType>::mergeBlock(
    const FeatureBlock<FeatureType>& other_block) {
  CHECK_EQ(other_block.block_size(), block_size());

  if (!other_block.has_data()) {
    return;
  } else {
    has_data() = true;
    updated() = true;

    for (const FeatureType& other_feature : other_block.features()) {
      features_.push_back(other_feature);
    }
  }
}

template <typename FeatureType>
size_t FeatureBlock<FeatureType>::getMemorySize() const {
  size_t size = 0u;

  // Calculate size of members
  size += sizeof(block_size_inv_);
  size += sizeof(origin_);
  size += sizeof(block_size_);

  size += sizeof(has_data_);
  size += sizeof(updated_);

  if (numFeatures() > 0u) {
    size += (numFeatures() * sizeof(FeatureType));
  }
  return size;
}

}  // namespace voxblox

#endif  // GLOBAL_FEATURE_MAP_FEATURE_BLOCK_INL_H_
