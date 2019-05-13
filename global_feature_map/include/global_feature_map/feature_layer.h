#ifndef GLOBAL_FEATURE_MAP_FEATURE_LAYER_H_
#define GLOBAL_FEATURE_MAP_FEATURE_LAYER_H_

#include <memory>
#include <string>
#include <utility>

#include <glog/logging.h>
#include <modelify_msgs/FeatureLayer.h>
#include <voxblox/core/block_hash.h>
#include <voxblox/core/common.h>

#include "./FeatureBlock.pb.h"
#include "./FeatureLayer.pb.h"
#include "global_feature_map/feature_block.h"

namespace voxblox {

enum class DeserializeAction : size_t {
  kUpdate = 0u,
  kMerge = 1u,
  kReset = 2u
};

enum class FeatureBlockMergingStrategy {
  kProhibit,
  kReplace,
  kDiscard,
  kMerge
};

/**
 * A 3D information layer, containing blocks with a set of features inside them.
 */
template <typename FeatureType>
class FeatureLayer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<FeatureLayer> Ptr;
  typedef FeatureBlock<FeatureType> FeatureBlockType;
  typedef typename AnyIndexHashMapType<typename FeatureBlockType::Ptr>::type
      FeatureBlockHashMap;
  typedef typename std::pair<BlockIndex, typename FeatureBlockType::Ptr>
      FeatureBlockMapPair;

  explicit FeatureLayer(FloatingPoint block_size) : block_size_(block_size) {
    CHECK_GT(block_size_, 0.0f);
    block_size_inv_ = 1.0 / block_size_;
    LOG(WARNING) << "Descriptor size was not provided, setting it to 0!";
    feature_descriptor_size_ = 0u;
  }
  explicit FeatureLayer(FloatingPoint block_size, size_t descriptor_size)
      : block_size_(block_size), feature_descriptor_size_(descriptor_size) {
    CHECK_GT(block_size_, 0.0f);
    block_size_inv_ = 1.0 / block_size_;
  }

  explicit FeatureLayer(const FeatureLayerProto& proto);

  /// Deep copy constructor.
  explicit FeatureLayer(const FeatureLayer& other);

  virtual ~FeatureLayer() {}

  inline const FeatureBlockType& getBlockByIndex(
      const BlockIndex& index) const {
    typename FeatureBlockHashMap::const_iterator it = block_map_.find(index);
    if (it == block_map_.end()) {
      LOG(FATAL) << "Accessed unallocated block at " << index.transpose();
    }
    return *(it->second);
  }

  inline FeatureBlockType& getBlockByIndex(const BlockIndex& index) {
    typename FeatureBlockHashMap::iterator it = block_map_.find(index);
    if (it == block_map_.end()) {
      LOG(FATAL) << "Accessed unallocated block at " << index.transpose();
    }
    return *(it->second);
  }

  inline typename FeatureBlockType::ConstPtr getBlockPtrByIndex(
      const BlockIndex& index) const {
    typename FeatureBlockHashMap::const_iterator it = block_map_.find(index);
    if (it != block_map_.end()) {
      return it->second;
    } else {
      return typename FeatureBlockType::ConstPtr();
    }
  }

  inline typename FeatureBlockType::Ptr getBlockPtrByIndex(
      const BlockIndex& index) {
    typename FeatureBlockHashMap::iterator it = block_map_.find(index);
    if (it != block_map_.end()) {
      return it->second;
    } else {
      return typename FeatureBlockType::Ptr();
    }
  }

  /**
   *  Gets a block by the block index if it already exists, otherwise allocates
   * a new one.
   */
  inline typename FeatureBlockType::Ptr allocateBlockPtrByIndex(
      const BlockIndex& index) {
    typename FeatureBlockHashMap::iterator it = block_map_.find(index);
    if (it != block_map_.end()) {
      return it->second;
    } else {
      return allocateNewBlock(index);
    }
  }

  inline typename FeatureBlockType::ConstPtr getBlockPtrByCoordinates(
      const Point& coords) const {
    return getBlockPtrByIndex(computeBlockIndexFromCoordinates(coords));
  }

  inline typename FeatureBlockType::Ptr getBlockPtrByCoordinates(
      const Point& coords) {
    return getBlockPtrByIndex(computeBlockIndexFromCoordinates(coords));
  }

  /**
   * Gets a block by the coordinates if it already exists,
   * otherwise allocates a new one.
   */
  inline typename FeatureBlockType::Ptr allocateBlockPtrByCoordinates(
      const Point& coords) {
    return allocateBlockPtrByIndex(computeBlockIndexFromCoordinates(coords));
  }

  /**
   * IMPORTANT NOTE: Due the limited accuracy of the FloatingPoint type, this
   * function doesn't always compute the correct block index for coordinates
   * near the block boundaries.
   */
  inline BlockIndex computeBlockIndexFromCoordinates(
      const Point& coords) const {
    return getGridIndexFromPoint<BlockIndex>(coords, block_size_inv_);
  }

  inline typename FeatureBlockType::Ptr allocateNewBlock(
      const BlockIndex& index) {
    auto insert_status = block_map_.emplace(
        index,
        std::make_shared<FeatureBlockType>(
            block_size_, getOriginPointFromGridIndex(index, block_size_)));

    DCHECK(insert_status.second)
        << "Block already exists when allocating at " << index.transpose();

    DCHECK(insert_status.first->second);
    DCHECK_EQ(insert_status.first->first, index);
    return insert_status.first->second;
  }

  inline typename FeatureBlockType::Ptr allocateNewBlockByCoordinates(
      const Point& coords) {
    return allocateNewBlock(computeBlockIndexFromCoordinates(coords));
  }

  inline void insertBlock(
      const std::pair<const BlockIndex,
                      typename FeatureBlock<FeatureType>::Ptr>& block_pair) {
    auto insert_status = block_map_.insert(block_pair);

    DCHECK(insert_status.second) << "Block already exists when inserting at "
                                 << insert_status.first->first.transpose();

    DCHECK(insert_status.first->second);
  }

  inline void removeBlock(const BlockIndex& index) { block_map_.erase(index); }
  inline void removeAllBlocks() { block_map_.clear(); }

  inline void removeBlockByCoordinates(const Point& coords) {
    block_map_.erase(computeBlockIndexFromCoordinates(coords));
  }

  inline void removeDistantBlocks(const Point& center,
                                  const double max_distance) {
    AlignedVector<BlockIndex> needs_erasing;
    for (const std::pair<const BlockIndex, typename FeatureBlockType::Ptr>& kv :
         block_map_) {
      if ((kv.second->origin() - center).squaredNorm() >
          max_distance * max_distance) {
        needs_erasing.push_back(kv.first);
      }
    }
    for (const BlockIndex& index : needs_erasing) {
      block_map_.erase(index);
    }
  }

  inline void getAllAllocatedBlocks(BlockIndexList* blocks) const {
    CHECK_NOTNULL(blocks);
    blocks->clear();
    blocks->reserve(block_map_.size());
    for (const std::pair<const BlockIndex, typename FeatureBlockType::Ptr>& kv :
         block_map_) {
      blocks->emplace_back(kv.first);
    }
  }

  inline void getAllUpdatedBlocks(BlockIndexList* blocks) const {
    CHECK_NOTNULL(blocks);
    blocks->clear();
    for (const std::pair<const BlockIndex, typename FeatureBlockType::Ptr>& kv :
         block_map_) {
      if (kv.second->updated()) {
        blocks->emplace_back(kv.first);
      }
    }
  }

  inline size_t getNumberOfAllocatedBlocks() const { return block_map_.size(); }

  inline bool hasBlock(const BlockIndex& block_index) const {
    return block_map_.count(block_index) > 0;
  }

  inline FloatingPoint block_size() const { return block_size_; }
  inline FloatingPoint block_size_inv() const { return block_size_inv_; }

  inline void getFeatures(std::vector<FeatureType>* features) const {
    CHECK_NOTNULL(features);

    BlockIndexList block_list;
    getAllAllocatedBlocks(&block_list);

    for (const BlockIndex& block_idx : block_list) {
      const typename FeatureBlock<FeatureType>::ConstPtr block =
          getBlockPtrByIndex(block_idx);
      CHECK_NOTNULL(block);

      const std::vector<FeatureType>& block_features = block->getFeatures();

      features->insert(features->end(), block_features.begin(),
                       block_features.end());
    }
  }

  inline size_t getNumberOfFeatures() const {
    BlockIndexList block_list;
    getAllAllocatedBlocks(&block_list);

    size_t number_of_features = 0u;
    for (const BlockIndex& block_idx : block_list) {
      const typename FeatureBlock<FeatureType>::ConstPtr block =
          getBlockPtrByIndex(block_idx);
      CHECK_NOTNULL(block);

      number_of_features += block->numFeatures();
    }
    return number_of_features;
  }

  size_t getMemorySize() const;

  inline void setDescriptorSize(const size_t size) {
    feature_descriptor_size_ = size;
  }
  inline size_t getDescriptorSize() const { return feature_descriptor_size_; }

  void serializeLayerAsMsg(const bool only_updated,
                           const DeserializeAction& action,
                           modelify_msgs::FeatureLayer* msg);

  bool deserializeMsgToLayer(const modelify_msgs::FeatureLayer& msg);
  bool deserializeMsgToLayer(const modelify_msgs::FeatureLayer& msg,
                             const DeserializeAction& action);

  // Serialization tools.
  void getProto(FeatureLayerProto* proto) const;
  bool isCompatible(const FeatureLayerProto& layer_proto) const;
  bool isCompatible(const FeatureBlockProto& layer_proto) const;
  bool saveToFile(const std::string& file_path, bool clear_file = true) const;
  bool saveSubsetToFile(const std::string& file_path,
                        BlockIndexList blocks_to_include,
                        bool include_all_blocks, bool clear_file = true) const;
  bool saveBlocksToStream(bool include_all_blocks,
                          BlockIndexList blocks_to_include,
                          std::fstream* outfile_ptr) const;
  bool addBlockFromProto(const FeatureBlockProto& block_proto,
                         FeatureBlockMergingStrategy strategy);

 protected:
  std::string getType() const;

  FloatingPoint block_size_;
  FloatingPoint block_size_inv_;
  size_t feature_descriptor_size_;

  FeatureBlockHashMap block_map_;
};

}  // namespace voxblox

#endif  // GLOBAL_FEATURE_MAP_FEATURE_LAYER_H_

#include "global_feature_map/feature_layer_inl.h"
