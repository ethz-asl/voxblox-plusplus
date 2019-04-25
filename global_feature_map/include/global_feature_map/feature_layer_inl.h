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

    new_block->getFeatures() = block_ptr->getFeatures();
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

template <typename FeatureType>
void FeatureLayer<FeatureType>::serializeLayerAsMsg(
    const bool only_updated, modelify_msgs::FeatureLayer* msg,
    const DeserializeAction& action) {
  CHECK_NOTNULL(msg);
  msg->block_size = block_size();
  msg->layer_type = getType();

  BlockIndexList block_list;
  if (only_updated) {
    getAllUpdatedBlocks(&block_list);
  } else {
    getAllAllocatedBlocks(&block_list);
  }

  msg->action = static_cast<uint8_t>(action);

  modelify_msgs::FeatureBlock block_msg;
  msg->blocks.reserve(block_list.size());
  for (const BlockIndex& index : block_list) {
    block_msg.x_index = index.x();
    block_msg.y_index = index.y();
    block_msg.z_index = index.z();

    std::vector<uint32_t> data;
    getBlockByIndex(index).serializeToIntegers(&data);

    block_msg.data = data;
    msg->blocks.push_back(block_msg);
  }
}

template <typename FeatureType>
bool FeatureLayer<FeatureType>::deserializeMsgToLayer(
    const modelify_msgs::FeatureLayer& msg) {
  return deserializeMsgToLayer(msg, static_cast<DeserializeAction>(msg.action));
}

template <typename FeatureType>
bool FeatureLayer<FeatureType>::deserializeMsgToLayer(
    const modelify_msgs::FeatureLayer& msg, const DeserializeAction& action) {
  if (getType().compare(msg.layer_type) != 0) {
    LOG(ERROR) << "Feature type does not match!";
    return false;
  }

  if (msg.block_size != block_size()) {
    LOG(ERROR) << "Sizes don't match!";
    return false;
  }

  if (action == DeserializeAction::kReset) {
    removeAllBlocks();
  }

  for (const modelify_msgs::FeatureBlock& block_msg : msg.blocks) {
    BlockIndex index(block_msg.x_index, block_msg.y_index, block_msg.z_index);

    // Either we want to update an existing block or there was no block there
    // before.
    if (action == DeserializeAction::kUpdate || !hasBlock(index)) {
      // Create a new block if it doesn't exist yet, or get the existing one
      // at the correct block index.
      typename FeatureBlock<FeatureType>::Ptr block_ptr =
          allocateBlockPtrByIndex(index);

      std::vector<uint32_t> data = block_msg.data;
      block_ptr->deserializeFromIntegers(data);

    } else if (action == DeserializeAction::kMerge) {
      typename FeatureBlock<FeatureType>::Ptr old_block_ptr =
          getBlockPtrByIndex(index);
      CHECK(old_block_ptr);

      typename FeatureBlock<FeatureType>::Ptr new_block_ptr(
          new FeatureBlock<FeatureType>(old_block_ptr->block_size(),
                                        old_block_ptr->origin()));

      std::vector<uint32_t> data = block_msg.data;
      new_block_ptr->deserializeFromIntegers(data);

      old_block_ptr->mergeBlock(*new_block_ptr);
    }
  }

  CHECK_GE(getNumberOfAllocatedBlocks(), msg.blocks.size());

  return true;
}

}  // namespace voxblox

#endif  // GLOBAL_FEATURE_MAP_FEATURE_LAYER_INL_H_
