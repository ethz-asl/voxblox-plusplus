#ifndef GLOBAL_FEATURE_MAP_FEATURE_UTILS_INL_H_
#define GLOBAL_FEATURE_MAP_FEATURE_UTILS_INL_H_

#include <fstream>
#include <memory>
#include <string>
#include <utility>

#include <glog/logging.h>
#include <voxblox/core/common.h>

#include "./Block.pb.h"
#include "./Layer.pb.h"

namespace voxblox {

template <typename FeatureType>
void centerBlocksOfFeatureLayer(FeatureLayer<FeatureType>* layer,
                                Point* new_layer_origin) {
  CHECK_NOTNULL(layer);

  // Compute the exact cenroid of all allocated block indices.
  Point centroid = Point::Zero();
  BlockIndexList block_indices;
  layer->getAllAllocatedBlocks(&block_indices);
  for (const BlockIndex block_index : block_indices) {
    centroid += layer->getBlockByIndex(block_index).origin();
  }
  centroid /= static_cast<FloatingPoint>(block_indices.size());

  // Round to nearest block index to centroid.
  centroid /= layer->block_size();
  // TODO(ntonci): Somethin is weird here, however, will not change anything
  // since then it will no longer give the same result as the one for tsdf
  // layer.
  const BlockIndex index_centroid =
      (centroid + 0.5 * Point::Ones()).cast<IndexElement>();

  // Return the new origin expressed in the old origins coordinate frame.
  const FloatingPoint block_size = layer->block_size();
  *new_layer_origin = index_centroid.cast<FloatingPoint>() * block_size;

  LOG(INFO) << "The new origin of the coordinate frame (expressed in the old "
            << "coordinate frame) is: " << new_layer_origin->transpose();

  // Loop over all blocks and change their spatial indices.
  // The only way to do this is to remove them all, store them in a temporary
  // hash map and re-insert them again. This sounds worse than it is, the blocks
  // are all shared ptrs and therefore only a negligible amount of real memory
  // operations is necessary.
  const size_t num_allocated_blocks_before =
      layer->getNumberOfAllocatedBlocks();
  typename FeatureLayer<FeatureType>::FeatureBlockHashMap temporary_map;
  for (const BlockIndex& block_index : block_indices) {
    typename FeatureBlock<FeatureType>::Ptr block_ptr =
        layer->getBlockPtrByIndex(block_index);
    block_ptr->shiftBlockWithFeatures(*new_layer_origin);

    // Extract block and shift block index.
    temporary_map.emplace(block_index - index_centroid, block_ptr);
  }

  layer->removeAllBlocks();
  CHECK_EQ(layer->getNumberOfAllocatedBlocks(), 0u);

  // Insert into layer again.
  for (const std::pair<const BlockIndex,
                       typename FeatureBlock<FeatureType>::Ptr>&
           idx_block_pair : temporary_map) {
    layer->insertBlock(idx_block_pair);
  }
  CHECK_EQ(layer->getNumberOfAllocatedBlocks(), num_allocated_blocks_before);
}

template <typename FeatureType>
void mergeFeatureLayerAintoFeatureLayerB(
    const FeatureLayer<FeatureType>& layer_A, const Transformation& T_B_A,
    FeatureLayer<FeatureType>* layer_B) {
  CHECK_NOTNULL(layer_B);
  FeatureLayer<FeatureType> layer_A_transformed(layer_B->block_size(),
                                                layer_B->getDescriptorSize());

  transformFeatureLayer(layer_A, T_B_A, &layer_A_transformed);

  mergeFeatureLayerAintoFeatureLayerB(layer_A_transformed, layer_B);
}

template <typename FeatureType>
void mergeFeatureLayerAintoFeatureLayerB(
    const FeatureLayer<FeatureType>& layer_A,
    FeatureLayer<FeatureType>* layer_B) {
  CHECK_NOTNULL(layer_B);
  const FeatureLayer<FeatureType>* layer_A_ptr = &layer_A;

  BlockIndexList block_idx_list_A;
  layer_A.getAllAllocatedBlocks(&block_idx_list_A);

  for (const BlockIndex& block_idx : block_idx_list_A) {
    typename FeatureBlock<FeatureType>::ConstPtr block_A_ptr =
        layer_A_ptr->getBlockPtrByIndex(block_idx);
    typename FeatureBlock<FeatureType>::Ptr block_B_ptr =
        layer_B->getBlockPtrByIndex(block_idx);

    if (!block_B_ptr) {
      block_B_ptr = layer_B->allocateBlockPtrByIndex(block_idx);
    }

    if ((block_A_ptr != nullptr) && (block_B_ptr != nullptr)) {
      block_B_ptr->mergeBlock(*block_A_ptr);
    }
  }
}

template <typename FeatureType>
void transformFeatureLayer(const FeatureLayer<FeatureType>& layer_in,
                           const Transformation& T_out_in,
                           FeatureLayer<FeatureType>* layer_out) {
  CHECK_NOTNULL(layer_out);

  // TODO(ntonci): Make config a param.
  FeatureIntegrator::Config feature_integrator_config;
  feature_integrator_config.match_descriptors_to_reject_duplicates = false;
  FeatureIntegrator feature_integrator(feature_integrator_config, layer_out);

  std::vector<FeatureType> features;
  layer_in.getFeatures(&features);

  feature_integrator.integrateFeatures(T_out_in, features);
}

template <typename FeatureType>
bool loadFeatureBlocksFromFile(const std::string& file_path,
                               FeatureBlockMergingStrategy strategy,
                               bool multiple_layer_support,
                               FeatureLayer<FeatureType>* layer_ptr) {
  CHECK_NOTNULL(layer_ptr);
  CHECK(!file_path.empty());

  // Open and check the file
  std::fstream proto_file;
  proto_file.open(file_path, std::fstream::in);
  if (!proto_file.is_open()) {
    LOG(ERROR) << "Could not open protobuf file to load layer: " << file_path;
    return false;
  }

  // Byte offset result, used to keep track where we are in the file if
  // necessary.
  uint32_t tmp_byte_offset = 0;

  bool layer_found = false;

  do {
    // Get number of messages
    uint32_t num_protos;
    if (!utils::readProtoMsgCountToStream(&proto_file, &num_protos,
                                          &tmp_byte_offset)) {
      LOG(ERROR) << "Could not read number of messages.";
      return false;
    }

    if (num_protos == 0u) {
      LOG(WARNING) << "Empty protobuf file!";
      return false;
    }

    // Get header and check if it is compatible with existing layer.
    FeatureLayerProto layer_proto;
    if (!utils::readProtoMsgFromStream(&proto_file, &layer_proto,
                                       &tmp_byte_offset)) {
      LOG(ERROR) << "Could not read layer protobuf message.";
      return false;
    }

    if (layer_ptr->isCompatible(layer_proto)) {
      layer_found = true;
    } else if (!multiple_layer_support) {
      LOG(ERROR)
          << "The layer information read from file is not compatible with "
             "the current layer!";
      return false;
    } else {
      // Figure out how much offset to jump forward by. This is the number of
      // blocks * the block size... Actually maybe we just read them in? This
      // is probably easiest. Then if there's corrupted blocks, we abort.
      // We just don't add these to the layer.
      const size_t num_blocks = num_protos - 1;
      for (uint32_t block_idx = 0u; block_idx < num_blocks; ++block_idx) {
        FeatureBlockProto block_proto;
        if (!utils::readProtoMsgFromStream(&proto_file, &block_proto,
                                           &tmp_byte_offset)) {
          LOG(ERROR) << "Could not read block protobuf message number "
                     << block_idx;
          return false;
        }
      }
      continue;
    }

    // Read all blocks and add them to the layer.
    const size_t num_blocks = num_protos - 1;
    if (!loadFeatureBlocksFromStream(num_blocks, strategy, &proto_file,
                                     layer_ptr, &tmp_byte_offset)) {
      return false;
    }
  } while (multiple_layer_support && !layer_found && !proto_file.eof());
  return layer_found;
}

template <typename FeatureType>
bool loadFeatureBlocksFromFile(const std::string& file_path,
                               FeatureBlockMergingStrategy strategy,
                               FeatureLayer<FeatureType>* layer_ptr) {
  constexpr bool multiple_layer_support = false;
  return LoadBlocksFromFile(file_path, strategy, multiple_layer_support,
                            layer_ptr);
}

template <typename FeatureType>
bool loadFeatureBlocksFromStream(const size_t num_blocks,
                                 FeatureBlockMergingStrategy strategy,
                                 std::fstream* proto_file_ptr,
                                 FeatureLayer<FeatureType>* layer_ptr,
                                 uint32_t* tmp_byte_offset_ptr) {
  CHECK_NOTNULL(proto_file_ptr);
  CHECK_NOTNULL(layer_ptr);
  CHECK_NOTNULL(tmp_byte_offset_ptr);
  // Read all blocks and add them to the layer.
  for (uint32_t block_idx = 0u; block_idx < num_blocks; ++block_idx) {
    FeatureBlockProto block_proto;
    if (!utils::readProtoMsgFromStream(proto_file_ptr, &block_proto,
                                       tmp_byte_offset_ptr)) {
      LOG(ERROR) << "Could not read block protobuf message number "
                 << block_idx;
      return false;
    }

    if (!layer_ptr->addBlockFromProto(block_proto, strategy)) {
      LOG(ERROR) << "Could not add the block protobuf message to the layer!";
      return false;
    }
  }
  return true;
}

template <typename FeatureType>
bool loadFeatureLayer(const std::string& file_path,
                      const bool multiple_layer_support,
                      typename FeatureLayer<FeatureType>::Ptr* layer_ptr) {
  CHECK_NOTNULL(layer_ptr);
  CHECK(!file_path.empty());

  // Open and check the file
  std::fstream proto_file;
  proto_file.open(file_path, std::fstream::in);
  if (!proto_file.is_open()) {
    LOG(ERROR) << "Could not open protobuf file to load layer: " << file_path;
    return false;
  }

  // Byte offset result, used to keep track where we are in the file if
  // necessary.
  uint32_t tmp_byte_offset = 0;

  bool layer_found = false;

  do {
    // Get number of messages
    uint32_t num_protos;
    if (!utils::readProtoMsgCountToStream(&proto_file, &num_protos,
                                          &tmp_byte_offset)) {
      LOG(ERROR) << "Could not read number of messages.";
      return false;
    }

    if (num_protos == 0u) {
      LOG(WARNING) << "Empty protobuf file!";
      return false;
    }

    // Get header and check if it is compatible with existing layer.
    FeatureLayerProto layer_proto;
    if (!utils::readProtoMsgFromStream(&proto_file, &layer_proto,
                                       &tmp_byte_offset)) {
      LOG(ERROR) << "Could not read layer protobuf message.";
      return false;
    }

    if (layer_proto.block_size() <= 0.0f) {
      LOG(ERROR)
          << "Invalid parameter in layer protobuf message. Check the format.";
      return false;
    }

    if (getFeatureType<FeatureType>().compare(layer_proto.type()) == 0) {
      layer_found = true;
    } else if (!multiple_layer_support) {
      LOG(ERROR)
          << "The layer information read from file is not compatible with "
             "the current layer!";
      return false;
    } else {
      // Figure out how much offset to jump forward by. This is the number of
      // blocks * the block size... Actually maybe we just read them in? This
      // is probably easiest. Then if there's corrupted blocks, we abort.
      // We just don't add these to the layer.
      const size_t num_blocks = num_protos - 1;
      for (uint32_t block_idx = 0u; block_idx < num_blocks; ++block_idx) {
        FeatureBlockProto block_proto;
        if (!utils::readProtoMsgFromStream(&proto_file, &block_proto,
                                           &tmp_byte_offset)) {
          LOG(ERROR) << "Could not read block protobuf message number "
                     << block_idx;
          return false;
        }
      }
      continue;
    }

    *layer_ptr = aligned_shared<FeatureLayer<FeatureType> >(layer_proto);
    CHECK(*layer_ptr);

    // Read all blocks and add them to the layer.
    const size_t num_blocks = num_protos - 1;
    if (!loadFeatureBlocksFromStream(
            num_blocks, FeatureBlockMergingStrategy::kProhibit, &proto_file,
            (*layer_ptr).get(), &tmp_byte_offset)) {
      return false;
    }
  } while (multiple_layer_support && !layer_found && !proto_file.eof());
  return layer_found;
}

template <typename FeatureType>
bool loadFeatureLayer(const std::string& file_path,
                      typename FeatureLayer<FeatureType>::Ptr* layer_ptr) {
  constexpr bool multiple_layer_support = false;
  return loadFeatureLayer<FeatureType>(file_path, multiple_layer_support,
                                       layer_ptr);
}

template <typename FeatureType>
bool saveFeatureLayer(const FeatureLayer<FeatureType>& layer,
                      const std::string& file_path, bool clear_file) {
  CHECK(!file_path.empty());
  return layer.saveToFile(file_path, clear_file);
}

template <typename FeatureType>
bool saveFeatureLayerSubset(const FeatureLayer<FeatureType>& layer,
                            const std::string& file_path,
                            const BlockIndexList& blocks_to_include,
                            bool include_all_blocks) {
  CHECK(!file_path.empty());
  return layer.saveSubsetToFile(file_path, blocks_to_include,
                                include_all_blocks);
}

}  // namespace voxblox

#endif  // GLOBAL_FEATURE_MAP_FEATURE_UTILS_INL_H_
