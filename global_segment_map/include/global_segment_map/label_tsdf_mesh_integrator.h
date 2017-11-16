#ifndef GLOBAL_SEGMENT_MAP_LABEL_TSDF_MESH_INTEGRATOR_H_
#define GLOBAL_SEGMENT_MAP_LABEL_TSDF_MESH_INTEGRATOR_H_

#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

#include <glog/logging.h>
#include <voxblox/core/color.h>
#include <voxblox/mesh/mesh_integrator.h>

#include "global_segment_map/label_voxel.h"

namespace voxblox {

class MeshLabelIntegrator : public MeshIntegrator<TsdfVoxel> {
 public:
  MeshLabelIntegrator(const MeshIntegrator::Config& config,
                      Layer<TsdfVoxel>* tsdf_layer,
                      Layer<LabelVoxel>* label_layer, MeshLayer* mesh_layer,
                      bool visualize_confidence = false)
      : MeshIntegrator(config, tsdf_layer, mesh_layer),
        label_layer_(CHECK_NOTNULL(label_layer)),
        visualize_confidence(visualize_confidence) {}

  // Generates mesh for the tsdf layer.
  void generateMesh(bool only_mesh_updated_blocks, bool clear_updated_flag) {
    BlockIndexList all_tsdf_blocks;
    BlockIndexList all_label_blocks;
    if (only_mesh_updated_blocks) {
      tsdf_layer_->getAllUpdatedBlocks(&all_tsdf_blocks);
      // TODO(grinvalm) unique union of block indices here.
      label_layer_->getAllUpdatedBlocks(&all_label_blocks);
    } else {
      tsdf_layer_->getAllAllocatedBlocks(&all_tsdf_blocks);
    }

    // Allocate all the mesh memory
    for (const BlockIndex& block_index : all_tsdf_blocks) {
      mesh_layer_->allocateMeshPtrByIndex(block_index);
    }

    ThreadSafeIndex index_getter(all_tsdf_blocks.size(),
                                 config_.integrator_threads);

    AlignedVector<std::thread> integration_threads;
    for (size_t i = 0; i < config_.integrator_threads; ++i) {
      integration_threads.emplace_back(
          &MeshIntegrator::generateMeshBlocksFunction, this, all_tsdf_blocks,
          clear_updated_flag, &index_getter);
    }

    for (std::thread& thread : integration_threads) {
      thread.join();
    }
  }

  void generateMeshBlocksFunction(const BlockIndexList& all_tsdf_blocks,
                                  bool clear_updated_flag,
                                  ThreadSafeIndex* index_getter) {
    DCHECK(index_getter != nullptr);

    size_t list_idx;
    while (index_getter->getNextIndex(&list_idx)) {
      const BlockIndex& block_idx = all_tsdf_blocks[list_idx];
      typename Block<TsdfVoxel>::Ptr tsdf_block =
          tsdf_layer_->getBlockPtrByIndex(block_idx);
      typename Block<LabelVoxel>::Ptr label_block =
          label_layer_->getBlockPtrByIndex(block_idx);

      updateMeshForBlock(block_idx);
      if (clear_updated_flag) {
        tsdf_block->updated() = false;
        label_block->updated() = false;
      }
    }
  }

  Color getColorFromLabel(const Label& label) {
    Color color;
    auto label_color_map_it = label_color_map_.find(label);

    if (label_color_map_it != label_color_map_.end()) {
      color = label_color_map_it->second;
    } else {
      color = randomColor();
      label_color_map_.insert(std::pair<Label, Color>(label, color));
    }
    return color;
  }

  virtual void updateMeshForBlock(const BlockIndex& block_index) {
    Mesh::Ptr mesh = mesh_layer_->getMeshPtrByIndex(block_index);
    mesh->clear();
    // This block should already exist, otherwise it makes no sense to update
    // the mesh for it. ;)
    Block<TsdfVoxel>::ConstPtr tsdf_block =
        tsdf_layer_->getBlockPtrByIndex(block_index);
    Block<LabelVoxel>::ConstPtr label_block =
        label_layer_->getBlockPtrByIndex(block_index);

    if (!tsdf_block && !label_block) {
      LOG(ERROR) << "Trying to mesh a non-existent block at index: "
                 << block_index.transpose();
      return;
    } else if (!(tsdf_block && label_block)) {
      LOG(FATAL) << "Block allocation differs between the two layers.";
    }

    extractBlockMesh(tsdf_block, mesh);
    // Update colors if needed.
    if (config_.use_color) {
      updateMeshColor(*label_block, mesh.get());
    }
  }

  void updateMeshColor(const Block<LabelVoxel>& label_block, Mesh* mesh) {
    DCHECK(mesh != nullptr);

    mesh->colors.clear();
    mesh->colors.resize(mesh->indices.size());

    // Use nearest-neighbor search.
    for (size_t i = 0; i < mesh->vertices.size(); ++i) {
      const Point& vertex = mesh->vertices[i];
      VoxelIndex voxel_index =
          label_block.computeVoxelIndexFromCoordinates(vertex);
      if (label_block.isValidVoxelIndex(voxel_index)) {
        const LabelVoxel& voxel = label_block.getVoxelByVoxelIndex(voxel_index);

        Color color;
        if (visualize_confidence) {
          // Scale values to range (0.0, 1.0).
          constexpr float expected_max_confidence = 100.0f;
          color =
              rainbowColorMap(voxel.label_confidence / expected_max_confidence);
        } else {
          color = getColorFromLabel(voxel.label);
        }
        mesh->colors[i] = color;
      } else {
        const typename Block<LabelVoxel>::ConstPtr neighbor_block =
            label_layer_->getBlockPtrByCoordinates(vertex);
        const LabelVoxel& voxel = neighbor_block->getVoxelByCoordinates(vertex);
        Color color;
        if (visualize_confidence) {
          color = rainbowColorMap(voxel.label_confidence / 100);
        } else {
          color = getColorFromLabel(voxel.label);
        }
        mesh->colors[i] = color;
      }
    }
  }

 protected:
  Layer<LabelVoxel>* label_layer_;

  // Flag to choose whether the mesh color
  // encodes the labels or the label confidences.
  // By default the mesh encodes the labels.
  bool visualize_confidence;

  std::map<Label, Color> label_color_map_;
};

}  // namespace voxblox

#endif  // GLOBAL_SEGMENT_MAP_LABEL_TSDF_MESH_INTEGRATOR_H_
