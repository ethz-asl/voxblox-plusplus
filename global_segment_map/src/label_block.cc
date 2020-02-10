#include <memory>

#include <glog/logging.h>

#include "global_segment_map/label_block.h"

namespace voxblox {

template <>
void Block<LabelVoxel>::mergeBlock(const Block<LabelVoxel>& other_block) {
  CHECK_EQ(other_block.voxel_size(), voxel_size());
  CHECK_EQ(other_block.voxels_per_side(), voxels_per_side());

  if (other_block.has_data()) {
    has_data() = true;
    // Do not set updated() to true, since when meshing we do not want duplicate
    // block indices across updated TSDF and Label blocks.

    for (IndexElement voxel_idx = 0;
         voxel_idx < static_cast<IndexElement>(num_voxels()); ++voxel_idx) {
      mergeVoxelAIntoVoxelB(other_block.getVoxelByLinearIndex(voxel_idx),
                            &(getVoxelByLinearIndex(voxel_idx)));
    }
  }
}

template <>
void Block<LabelVoxel>::deserializeFromIntegers(
    const std::vector<uint32_t>& data) {
  constexpr size_t kNumDataPacketsPerVoxel = 2u;
  const size_t num_data_packets = data.size();
  CHECK_EQ(num_voxels_ * kNumDataPacketsPerVoxel, num_data_packets);
  for (size_t voxel_idx = 0u, data_idx = 0u;
       voxel_idx < num_voxels_ && data_idx < num_data_packets;
       ++voxel_idx, data_idx += kNumDataPacketsPerVoxel) {
    const uint32_t bytes_1 = data[data_idx];
    const uint32_t bytes_2 = data[data_idx + 1u];

    LabelVoxel& voxel = voxels_[voxel_idx];

    memcpy(&(voxel.label_confidence), &bytes_1, sizeof(bytes_1));
    memcpy(&(voxel.label), &bytes_2, sizeof(bytes_2));
  }
}

// TODO(grinvalm): serialize and deserialize also votes array.
template <>
void Block<LabelVoxel>::serializeToIntegers(std::vector<uint32_t>* data) const {
  CHECK_NOTNULL(data);
  constexpr size_t kNumDataPacketsPerVoxel = 2u;
  data->clear();
  data->reserve(num_voxels_ * kNumDataPacketsPerVoxel);
  for (size_t voxel_idx = 0u; voxel_idx < num_voxels_; ++voxel_idx) {
    const LabelVoxel& voxel = voxels_[voxel_idx];

    const uint32_t* bytes_1_ptr =
        reinterpret_cast<const uint32_t*>(&voxel.label_confidence);
    data->push_back(*bytes_1_ptr);

    const uint32_t* bytes_2_ptr =
        reinterpret_cast<const uint32_t*>(&voxel.label);
    data->push_back(*bytes_2_ptr);
  }
  CHECK_EQ(num_voxels_ * kNumDataPacketsPerVoxel, data->size());
}

}  // namespace voxblox
