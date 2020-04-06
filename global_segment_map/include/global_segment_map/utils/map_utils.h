#ifndef GLOBAL_SEGMENT_MAP_UTILS_MAP_UTILS_H_
#define GLOBAL_SEGMENT_MAP_UTILS_MAP_UTILS_H_

#include <global_segment_map/common.h>
#include <global_segment_map/label_tsdf_map.h>
#include <global_segment_map/semantic_instance_label_fusion.h>

namespace voxblox {

inline void createPointcloudFromMap(const LabelTsdfMap& map,
                                    pcl::PointCloud<PointMapType>* pointcloud) {
  CHECK_NOTNULL(pointcloud);
  pointcloud->clear();

  const Layer<TsdfVoxel>& tsdf_layer = map.getTsdfLayer();
  const Layer<LabelVoxel>& label_layer = map.getLabelLayer();

  const SemanticInstanceLabelFusion semantic_instance_label_fusion =
      map.getSemanticInstanceLabelFusion();

  BlockIndexList blocks;
  tsdf_layer.getAllAllocatedBlocks(&blocks);

  // Cache layer settings.
  size_t vps = tsdf_layer.voxels_per_side();
  size_t num_voxels_per_block = vps * vps * vps;

  // Temp variables.
  double intensity = 0.0;
  // Iterate over all blocks.
  for (const BlockIndex& index : blocks) {
    const Block<TsdfVoxel>& tsdf_block = tsdf_layer.getBlockByIndex(index);
    const Block<LabelVoxel>& label_block = label_layer.getBlockByIndex(index);

    // Iterate over all voxels in said blocks.
    for (size_t linear_index = 0; linear_index < num_voxels_per_block;
         ++linear_index) {
      Point coord = tsdf_block.computeCoordinatesFromLinearIndex(linear_index);

      TsdfVoxel tsdf_voxel = tsdf_block.getVoxelByLinearIndex(linear_index);
      LabelVoxel label_voxel = label_block.getVoxelByLinearIndex(linear_index);

      constexpr float kMinWeight = 0.0f;
      constexpr float kFramesCountThresholdFactor = 0.1f;

      if (tsdf_voxel.weight > kMinWeight) {
        Label segment_label = label_voxel.label;
        SemanticLabel semantic_class = BackgroundLabel;
        InstanceLabel instance_id =
            semantic_instance_label_fusion.getInstanceLabel(
                segment_label, kFramesCountThresholdFactor);

        if (instance_id) {
          semantic_class = semantic_instance_label_fusion.getSemanticLabel(
              label_voxel.label);
        }

        PointMapType point;

        point.x = coord.x();
        point.y = coord.y();
        point.z = coord.z();

        point.distance = tsdf_voxel.distance;
        point.weight = tsdf_voxel.weight;

        point.segment_label = segment_label;
        point.semantic_class = semantic_class;
        pointcloud->push_back(point);
      }
    }
  }
}

}  // namespace voxblox

#endif  // GLOBAL_SEGMENT_MAP_UTILS_MAP_UTILS_H_
