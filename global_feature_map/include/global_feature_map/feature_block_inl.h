#ifndef GLOBAL_FEATURE_MAP_FEATURE_BLOCK_INL_H_
#define GLOBAL_FEATURE_MAP_FEATURE_BLOCK_INL_H_

#include <vector>

#include <voxblox/core/common.h>

namespace voxblox {

template <typename FeatureType>
FeatureBlock<FeatureType>::FeatureBlock(const FeatureBlockProto& proto,
                                        const size_t descriptor_size)
    : FeatureBlock(proto.block_size(), Point(proto.origin_x(), proto.origin_y(),
                                             proto.origin_z())) {
  has_data_ = proto.has_data();

  // Convert the data into a vector of integers.
  std::vector<uint32_t> data;
  data.reserve(proto.feature_data_size());

  for (uint32_t word : proto.feature_data()) {
    data.push_back(word);
  }

  deserializeFromIntegers(descriptor_size, data);
}

template <typename FeatureType>
void FeatureBlock<FeatureType>::mergeBlock(
    const FeatureBlock<FeatureType>& other_block) {
  CHECK_EQ(other_block.block_size(), block_size());

  if (!other_block.has_data()) {
    return;
  } else {
    has_data() = true;
    updated() = true;

    for (const FeatureType& other_feature : other_block.getFeatures()) {
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

  size += sizeof(block_mutex_);

  if (numFeatures() > 0u) {
    size += (numFeatures() * sizeof(FeatureType));
  }
  return size;
}

template <typename FeatureType>
void FeatureBlock<FeatureType>::getProto(const size_t descriptor_size,
                                         FeatureBlockProto* proto) const {
  CHECK_NOTNULL(proto);

  proto->set_block_size(block_size_);

  proto->set_origin_x(origin_.x());
  proto->set_origin_y(origin_.y());
  proto->set_origin_z(origin_.z());

  proto->set_has_data(has_data_);

  std::vector<uint32_t> data;
  serializeToIntegers(descriptor_size, &data);
  // Not quite actually a word since we're in a 64-bit age now, but whatever.
  for (uint32_t word : data) {
    proto->add_feature_data(word);
  }
}

}  // namespace voxblox

#endif  // GLOBAL_FEATURE_MAP_FEATURE_BLOCK_INL_H_
