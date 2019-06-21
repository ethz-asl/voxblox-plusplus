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

#include "./FeatureBlock.pb.h"
#include "./FeatureLayer.pb.h"
#include "global_feature_map/feature_types.h"

namespace voxblox {

template <typename FeatureType>
FeatureLayer<FeatureType>::FeatureLayer(const FeatureLayerProto& proto)
    : block_size_(proto.block_size()),
      feature_descriptor_size_(proto.feature_descriptor_size()) {
  CHECK_EQ(getType().compare(proto.type()), 0)
      << "Incorrect feature type, proto type: " << proto.type()
      << " layer type: " << getType();

  // Derived config parameter.
  CHECK_GT(block_size_, 0.0);
  block_size_inv_ = 1.0 / block_size_;
}

template <typename FeatureType>
FeatureLayer<FeatureType>::FeatureLayer(const FeatureLayer& other) {
  block_size_ = other.block_size_;
  block_size_inv_ = other.block_size_inv_;
  feature_descriptor_size_ = other.feature_descriptor_size_;

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
  size += sizeof(feature_descriptor_size_);

  for (const FeatureBlockMapPair& entry : block_map_) {
    size += entry.second->getMemorySize();
  }

  return size;
}

template <typename FeatureType>
std::string FeatureLayer<FeatureType>::getType() const {
  return getFeatureType<FeatureType>();
}

template <typename FeatureType>
void FeatureLayer<FeatureType>::serializeLayerAsMsg(
    const bool only_updated, const DeserializeAction& action,
    modelify_msgs::FeatureLayer* msg) {
  CHECK_NOTNULL(msg);
  if (getDescriptorSize() == 0u) {
    LOG(WARNING)
        << "You need to set the descriptor size before you can serialize the "
           "layer! You can do this either directly in the constructor of the "
           "layer or during feature callback.";
  }
  msg->block_size = block_size();
  msg->feature_descriptor_size = getDescriptorSize();
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
    getBlockByIndex(index).serializeToIntegers(getDescriptorSize(), &data);

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
  // TODO(ntonci): Maybe this check is not required in case you just want
  // keypoints without descriptors.
  CHECK_GT(getDescriptorSize(), 0u)
      << "You need to set the descriptor size before you can serialize the "
         "layer! You can do this either directly in the constructor of the "
         "layer or during feature callback.";

  if (getType().compare(msg.layer_type) != 0) {
    LOG(ERROR) << "Feature type does not match!";
    return false;
  }

  if (msg.block_size != block_size()) {
    LOG(ERROR) << "Sizes don't match!";
    return false;
  }

  if (msg.feature_descriptor_size != getDescriptorSize()) {
    LOG(ERROR) << "Descriptor sizes don't match!";
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
      block_ptr->deserializeFromIntegers(getDescriptorSize(), data);

    } else if (action == DeserializeAction::kMerge) {
      typename FeatureBlock<FeatureType>::Ptr old_block_ptr =
          getBlockPtrByIndex(index);
      CHECK(old_block_ptr);

      typename FeatureBlock<FeatureType>::Ptr new_block_ptr(
          new FeatureBlock<FeatureType>(old_block_ptr->block_size(),
                                        old_block_ptr->origin()));

      std::vector<uint32_t> data = block_msg.data;
      new_block_ptr->deserializeFromIntegers(getDescriptorSize(), data);

      old_block_ptr->mergeBlock(*new_block_ptr);
    }
  }

  CHECK_GE(getNumberOfAllocatedBlocks(), msg.blocks.size());

  return true;
}

template <typename FeatureType>
void FeatureLayer<FeatureType>::getProto(FeatureLayerProto* proto) const {
  CHECK_NOTNULL(proto);

  proto->set_block_size(block_size_);
  proto->set_feature_descriptor_size(feature_descriptor_size_);
  proto->set_type(getType());
}

template <typename FeatureType>
bool FeatureLayer<FeatureType>::isCompatible(
    const FeatureLayerProto& layer_proto) const {
  bool compatible = true;
  compatible &= (std::fabs(layer_proto.block_size() - block_size_) <
                 std::numeric_limits<FloatingPoint>::epsilon());
  compatible &= ((layer_proto.feature_descriptor_size() -
                  feature_descriptor_size_) == 0u);
  compatible &= (getType().compare(layer_proto.type()) == 0);

  if (!compatible) {
    LOG(WARNING) << "Block size of the loaded map is: "
                 << layer_proto.block_size()
                 << " but the current map is: " << block_size_
                 << " check passed? "
                 << (std::fabs(layer_proto.block_size() - block_size_) <
                     std::numeric_limits<FloatingPoint>::epsilon())
                 << "Descriptor size of the loaded map is: "
                 << layer_proto.feature_descriptor_size()
                 << " but the current map is: " << feature_descriptor_size_
                 << " check passed? "
                 << ((layer_proto.feature_descriptor_size() -
                      feature_descriptor_size_) == 0u)
                 << "\nFeatureLayer type of the loaded map is: " << getType()
                 << " but the current map is: " << layer_proto.type()
                 << " check passed? "
                 << (getType().compare(layer_proto.type()) == 0)
                 << "\nAre the maps using the same floating-point type? "
                 << (layer_proto.block_size() == block_size_) << std::endl;
  }
  return compatible;
}

template <typename FeatureType>
bool FeatureLayer<FeatureType>::isCompatible(
    const FeatureBlockProto& block_proto) const {
  bool compatible = true;
  compatible &= (std::fabs(block_proto.block_size() - block_size_) <
                 std::numeric_limits<FloatingPoint>::epsilon());
  return compatible;
}

template <typename FeatureType>
bool FeatureLayer<FeatureType>::saveToFile(const std::string& file_path,
                                           bool clear_file) const {
  constexpr bool kIncludeAllBlocks = true;
  return saveSubsetToFile(file_path, BlockIndexList(), kIncludeAllBlocks,
                          clear_file);
}

template <typename FeatureType>
bool FeatureLayer<FeatureType>::saveSubsetToFile(
    const std::string& file_path, BlockIndexList blocks_to_include,
    bool include_all_blocks, bool clear_file) const {
  CHECK(!file_path.empty());

  std::fstream outfile;
  // Will APPEND to the current file in case outputting multiple layers on the
  // same file, depending on the flag.
  std::ios_base::openmode file_flags = std::fstream::out | std::fstream::binary;
  if (!clear_file) {
    file_flags |= std::fstream::app | std::fstream::ate;
  } else {
    file_flags |= std::fstream::trunc;
  }
  outfile.open(file_path, file_flags);
  if (!outfile.is_open()) {
    LOG(ERROR) << "Could not open file for writing: " << file_path;
    return false;
  }

  // Only serialize the blocks if there are any.
  // Count the number of blocks that need to be serialized.
  size_t num_blocks_to_write = 0u;
  if ((include_all_blocks && !block_map_.empty()) ||
      !blocks_to_include.empty()) {
    for (const FeatureBlockMapPair& pair : block_map_) {
      bool write_block_to_file = include_all_blocks;

      if (!write_block_to_file) {
        BlockIndexList::const_iterator it = std::find(
            blocks_to_include.begin(), blocks_to_include.end(), pair.first);
        if (it != blocks_to_include.end()) {
          ++num_blocks_to_write;
        }
      } else {
        ++num_blocks_to_write;
      }
    }
  }
  if (include_all_blocks) {
    CHECK_EQ(num_blocks_to_write, block_map_.size());
  } else {
    CHECK_LE(num_blocks_to_write, block_map_.size());
    CHECK_LE(num_blocks_to_write, blocks_to_include.size());
  }

  // Write the total number of messages to the beginning of this file.
  // One layer header and then all the block maps
  const uint32_t num_messages = 1u + num_blocks_to_write;
  if (!utils::writeProtoMsgCountToStream(num_messages, &outfile)) {
    LOG(ERROR) << "Could not write message number to file.";
    outfile.close();
    return false;
  }

  // Write out the layer header.
  FeatureLayerProto proto_layer;
  getProto(&proto_layer);
  if (!utils::writeProtoMsgToStream(proto_layer, &outfile)) {
    LOG(ERROR) << "Could not write layer header message.";
    outfile.close();
    return false;
  }

  // Serialize blocks.
  saveBlocksToStream(include_all_blocks, blocks_to_include, &outfile);

  outfile.close();
  return true;
}

template <typename FeatureType>
bool FeatureLayer<FeatureType>::saveBlocksToStream(
    bool include_all_blocks, BlockIndexList blocks_to_include,
    std::fstream* outfile_ptr) const {
  CHECK_NOTNULL(outfile_ptr);
  for (const FeatureBlockMapPair& pair : block_map_) {
    bool write_block_to_file = include_all_blocks;
    if (!write_block_to_file) {
      BlockIndexList::const_iterator it = std::find(
          blocks_to_include.begin(), blocks_to_include.end(), pair.first);
      if (it != blocks_to_include.end()) {
        write_block_to_file = true;
      }
    }
    if (write_block_to_file) {
      FeatureBlockProto block_proto;
      pair.second->getProto(feature_descriptor_size_, &block_proto);

      if (!utils::writeProtoMsgToStream(block_proto, outfile_ptr)) {
        LOG(ERROR) << "Could not write block message.";
        return false;
      }
    }
  }
  return true;
}

template <typename FeatureType>
bool FeatureLayer<FeatureType>::addBlockFromProto(
    const FeatureBlockProto& block_proto,
    FeatureBlockMergingStrategy strategy) {
  if (isCompatible(block_proto)) {
    typename FeatureBlockType::Ptr block_ptr(
        new FeatureBlockType(block_proto, feature_descriptor_size_));
    const BlockIndex block_index = getGridIndexFromOriginPoint<BlockIndex>(
        block_ptr->origin(), block_size_inv_);
    switch (strategy) {
      case FeatureBlockMergingStrategy::kProhibit:
        CHECK_EQ(block_map_.count(block_index), 0u)
            << "Block collision at index: " << block_index;
        block_map_[block_index] = block_ptr;
        break;
      case FeatureBlockMergingStrategy::kReplace:
        block_map_[block_index] = block_ptr;
        break;
      case FeatureBlockMergingStrategy::kDiscard:
        block_map_.insert(std::make_pair(block_index, block_ptr));
        break;
      case FeatureBlockMergingStrategy::kMerge: {
        typename FeatureBlockHashMap::iterator it =
            block_map_.find(block_index);
        if (it == block_map_.end()) {
          block_map_[block_index] = block_ptr;
        } else {
          it->second->mergeBlock(*block_ptr);
        }
      } break;
      default:
        LOG(FATAL) << "Unknown FeatureBlockMergingStrategy: "
                   << static_cast<int>(strategy);
        return false;
    }
    // Mark that this block has been updated.
    block_map_[block_index]->updated() = true;
  } else {
    LOG(ERROR)
        << "The blocks from this protobuf are not compatible with this layer!";
    return false;
  }
  return true;
}

}  // namespace voxblox

#endif  // GLOBAL_FEATURE_MAP_FEATURE_LAYER_INL_H_
