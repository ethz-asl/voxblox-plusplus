#include "global_segment_map/label_tsdf_map.h"

namespace voxblox {

Labels LabelTsdfMap::getLabelList() {
  Labels labels;
  int count_unused_labels = 0;
  for (std::pair<Label, int> label_count_pair : label_count_map_) {
    if (label_count_pair.second > 0) {
      labels.push_back(label_count_pair.first);
    } else {
      count_unused_labels++;
    }
  }
  LOG(ERROR) << "Unused labels count: " << count_unused_labels;
  return labels;
}

InstanceLabels LabelTsdfMap::getInstanceList() {
  std::set<InstanceLabel> instance_labels_set;
  Labels labels = getLabelList();

  float kFramesCountThresholdFactor = 0.1f;

  for (Label label : labels) {
    InstanceLabel instance_label = instance_label_fusion_.getLabelInstance(
        label, kFramesCountThresholdFactor);
    if (instance_label != 0u) {
      instance_labels_set.emplace(instance_label);
    }
  }
  InstanceLabels instance_labels(instance_labels_set.begin(),
                                 instance_labels_set.end());

  return instance_labels;
}

void LabelTsdfMap::extractSegmentLayers(
    const std::vector<Label>& labels,
    std::unordered_map<Label, LayerPair>* label_layers_map,
    bool labels_list_is_complete) {
  CHECK(label_layers_map);

  // Map a label to its corresponding TSDF and label layers.
  Layer<TsdfVoxel> tsdf_layer_empty(config_.voxel_size,
                                    config_.voxels_per_side);
  Layer<LabelVoxel> label_layer_empty(config_.voxel_size,
                                      config_.voxels_per_side);
  for (const Label& label : labels) {
    label_layers_map->emplace(
        label, std::make_pair(tsdf_layer_empty, label_layer_empty));
  }

  BlockIndexList all_label_blocks;
  tsdf_layer_->getAllAllocatedBlocks(&all_label_blocks);

  for (const BlockIndex& block_index : all_label_blocks) {
    Block<TsdfVoxel>::Ptr global_tsdf_block =
        tsdf_layer_->getBlockPtrByIndex(block_index);
    Block<LabelVoxel>::Ptr global_label_block =
        label_layer_->getBlockPtrByIndex(block_index);

    const size_t vps = global_label_block->voxels_per_side();
    for (size_t i = 0; i < vps * vps * vps; ++i) {
      const LabelVoxel& global_label_voxel =
          global_label_block->getVoxelByLinearIndex(i);

      if (global_label_voxel.label == 0u) {
        continue;
      }

      auto it = label_layers_map->find(global_label_voxel.label);
      if (it == label_layers_map->end()) {
        if (labels_list_is_complete) {
          // TODO(margaritaG): this seemed to fail once, find out why there are
          // labels in the map which are not in the label list.
          LOG(FATAL) << "At least one voxel in the GSM is assigned to label "
                     << global_label_voxel.label
                     << " which is not in the given "
                        "list of labels to retrieve.";
        }
        continue;
      }

      Layer<TsdfVoxel>& tsdf_layer = it->second.first;
      Layer<LabelVoxel>& label_layer = it->second.second;

      Block<TsdfVoxel>::Ptr tsdf_block =
          tsdf_layer.allocateBlockPtrByIndex(block_index);
      Block<LabelVoxel>::Ptr label_block =
          label_layer.allocateBlockPtrByIndex(block_index);
      CHECK(tsdf_block);
      CHECK(label_block);

      TsdfVoxel& tsdf_voxel = tsdf_block->getVoxelByLinearIndex(i);
      LabelVoxel& label_voxel = label_block->getVoxelByLinearIndex(i);

      const TsdfVoxel& global_tsdf_voxel =
          global_tsdf_block->getVoxelByLinearIndex(i);

      tsdf_voxel = global_tsdf_voxel;
      label_voxel = global_label_voxel;
    }
  }
}

void LabelTsdfMap::extractInstanceLayers(
    const InstanceLabels& instance_labels,
    std::unordered_map<InstanceLabel, LayerPair>* instance_layers_map) {
  // Map an instance label to its corresponding TSDF and label layers.
  Layer<TsdfVoxel> tsdf_layer_empty(config_.voxel_size,
                                    config_.voxels_per_side);
  Layer<LabelVoxel> label_layer_empty(config_.voxel_size,
                                      config_.voxels_per_side);
  for (const InstanceLabel& instance_label : instance_labels) {
    instance_layers_map->emplace(
        instance_label, std::make_pair(tsdf_layer_empty, label_layer_empty));
  }

  BlockIndexList all_label_blocks;
  tsdf_layer_->getAllAllocatedBlocks(&all_label_blocks);

  for (const BlockIndex& block_index : all_label_blocks) {
    Block<TsdfVoxel>::Ptr global_tsdf_block =
        tsdf_layer_->getBlockPtrByIndex(block_index);
    Block<LabelVoxel>::Ptr global_label_block =
        label_layer_->getBlockPtrByIndex(block_index);

    const size_t vps = global_label_block->voxels_per_side();
    for (size_t i = 0; i < vps * vps * vps; ++i) {
      const LabelVoxel& global_label_voxel =
          global_label_block->getVoxelByLinearIndex(i);

      if (global_label_voxel.label == 0u) {
        continue;
      }
      float kFramesCountThresholdFactor = 0.1f;
      InstanceLabel instance_label = instance_label_fusion_.getLabelInstance(
          global_label_voxel.label, kFramesCountThresholdFactor);

      if (instance_label == 0u) {
        continue;
      }

      auto it = instance_layers_map->find(instance_label);
      if (it == instance_layers_map->end()) {
        continue;
      }

      Layer<TsdfVoxel>& tsdf_layer = it->second.first;
      Layer<LabelVoxel>& label_layer = it->second.second;

      Block<TsdfVoxel>::Ptr tsdf_block =
          tsdf_layer.allocateBlockPtrByIndex(block_index);
      Block<LabelVoxel>::Ptr label_block =
          label_layer.allocateBlockPtrByIndex(block_index);
      CHECK(tsdf_block);
      CHECK(label_block);

      TsdfVoxel& tsdf_voxel = tsdf_block->getVoxelByLinearIndex(i);
      LabelVoxel& label_voxel = label_block->getVoxelByLinearIndex(i);

      const TsdfVoxel& global_tsdf_voxel =
          global_tsdf_block->getVoxelByLinearIndex(i);

      tsdf_voxel = global_tsdf_voxel;
      label_voxel = global_label_voxel;
    }
  }
}

}  // namespace voxblox
