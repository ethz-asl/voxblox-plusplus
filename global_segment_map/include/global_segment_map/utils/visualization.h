#ifndef GLOBAL_SEGMENT_MAP_UTILS_VISUALIZATION_H_
#define GLOBAL_SEGMENT_MAP_UTILS_VISUALIZATION_H_

#include <voxblox/core/common.h>
#include <voxblox/core/voxel.h>

namespace voxblox {
inline void createLabelDistancePointcloudFromLabelTsdfLayer(
    const Layer<TsdfVoxel>& tsdf_layer, const Layer<LabelVoxel>& label_layer,
    pcl::PointCloud<pcl::PointXYZI>* pointcloud) {
  CHECK_NOTNULL(pointcloud);
  pointcloud->clear();
  BlockIndexList blocks;
  tsdf_layer.getAllAllocatedBlocks(&blocks);

  // Cache layer settings.
  size_t vps = tsdf_layer.voxels_per_side();
  size_t num_voxels_per_block = vps * vps * vps;

  // Temp variables.
  double intensity = 0.0;
  // Iterate over all blocks.
  for (const BlockIndex& index : blocks) {
    // Iterate over all voxels in said blocks.
    const Block<TsdfVoxel>& tsdf_block = tsdf_layer.getBlockByIndex(index);
    const Block<LabelVoxel>& label_block = label_layer.getBlockByIndex(index);

    for (size_t linear_index = 0; linear_index < num_voxels_per_block;
         ++linear_index) {
      Point coord = tsdf_block.computeCoordinatesFromLinearIndex(linear_index);
      TsdfVoxel tsdf_voxel = tsdf_block.getVoxelByLinearIndex(linear_index);
      LabelVoxel label_voxel = label_block.getVoxelByLinearIndex(linear_index);

      constexpr float kMinWeight = 0.0f;
      if (tsdf_voxel.weight > kMinWeight) {
        intensity = label_voxel.label;
        pcl::PointXYZI point;
        point.x = coord.x();
        point.y = coord.y();
        point.z = coord.z();
        point.intensity = intensity;
        pointcloud->push_back(point);
      }
    }
  }
}
}  // namespace voxblox
#endif  // GLOBAL_SEGMENT_MAP_UTILS_VISUALIZATION_H_
