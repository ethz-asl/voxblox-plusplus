#ifndef GLOBAL_FEATURE_MAP_FEATURE_LAYER_INL_H_
#define GLOBAL_FEATURE_MAP_FEATURE_LAYER_INL_H_

#include <fstream>  // NOLINT
#include <limits>
#include <string>
#include <utility>

#include <glog/logging.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/message.h>
#include <google/protobuf/message_lite.h>
#include <voxblox/core/block.h>
#include <voxblox/utils/protobuf_utils.h>

#include "global_feature_map/feature_types.h"

namespace voxblox {

template <typename FeatureType>
FeatureLayer<FeatureType>::FeatureLayer(const FeatureLayer& other) {
  block_size_ = other.block_size_;
  block_size_inv_ = other.block_size_inv_;

  for (const typename FeatureBlockHashMap::value_type& key_value_pair :
       other.block_map_) {
    const BlockIndex& block_idx = key_value_pair.first;
    const typename FeatureBlockType::Ptr& block_ptr = key_value_pair.second;
    CHECK(block_ptr);

    typename FeatureBlockType::Ptr new_block =
        allocateBlockPtrByIndex(block_idx);
    CHECK(new_block);

    *new_block = *block_ptr;
  }
}

template <typename FeatureType>
size_t FeatureLayer<FeatureType>::getMemorySize() const {
  size_t size = 0u;

  // Calculate size of members
  size += sizeof(block_size_);
  size += sizeof(block_size_inv_);

  // Calculate size of blocks
  size_t num_blocks = getNumberOfAllocatedBlocks();
  if (num_blocks > 0u) {
    typename FeatureBlock<FeatureType>::Ptr block = block_map_.begin()->second;
    size += num_blocks * block->getMemorySize();
  }
  return size;
}

template <typename FeatureType>
std::string FeatureLayer<FeatureType>::getType() const {
  return getFeatureType<FeatureType>();
}

}  // namespace voxblox

#endif  // GLOBAL_FEATURE_MAP_FEATURE_LAYER_INL_H_
