#ifndef GLOBAL_SEGMENT_MAP_LABEL_TSDF_MESH_INTEGRATOR_H_
#define GLOBAL_SEGMENT_MAP_LABEL_TSDF_MESH_INTEGRATOR_H_

#include <algorithm>
#include <cmath>
#include <list>
#include <map>
#include <vector>

#include <glog/logging.h>
#include <voxblox/core/color.h>
#include <voxblox/mesh/mesh_integrator.h>

#include "global_segment_map/label_voxel.h"

namespace voxblox {

class MeshLabelIntegrator : public MeshIntegrator<TsdfVoxel> {
 public:
  MeshLabelIntegrator(const MeshIntegratorConfig& config,
                      Layer<TsdfVoxel>* tsdf_layer,
                      Layer<LabelVoxel>* label_layer, MeshLayer* mesh_layer,
                      const std::map<Label, int>& label_age_map = {},
                      bool visualize_confidence = false)
      : MeshIntegrator(config, tsdf_layer, mesh_layer),
        label_layer_mutable_(CHECK_NOTNULL(label_layer)),
        label_layer_const_(CHECK_NOTNULL(label_layer)),
        label_age_map_ptr_(&label_age_map),
        visualize_confidence(visualize_confidence) {}

  MeshLabelIntegrator(const MeshIntegratorConfig& config,
                      const Layer<TsdfVoxel>& tsdf_layer,
                      const Layer<LabelVoxel>& label_layer,
                      MeshLayer* mesh_layer,
                      const std::map<Label, int>& label_age_map = {},
                      bool visualize_confidence = false)
      : MeshIntegrator(config, tsdf_layer, mesh_layer),
        label_layer_mutable_(nullptr),
        label_layer_const_(CHECK_NOTNULL(&label_layer)),
        label_age_map_ptr_(&label_age_map),
        visualize_confidence(visualize_confidence) {}

  // Generates mesh for the tsdf layer.
  bool generateMesh(bool only_mesh_updated_blocks, bool clear_updated_flag) {
    BlockIndexList all_tsdf_blocks;
    BlockIndexList all_label_blocks;
    if (only_mesh_updated_blocks) {
      sdf_layer_const_->getAllUpdatedBlocks(&all_tsdf_blocks);
      // TODO(grinvalm) unique union of block indices here.
      label_layer_const_->getAllUpdatedBlocks(&all_label_blocks);
      if (all_label_blocks.size() == 0u && all_tsdf_blocks.size() == 0u)
        return false;
    } else {
      sdf_layer_const_->getAllAllocatedBlocks(&all_tsdf_blocks);
    }

    // Allocate all the mesh memory
    for (const BlockIndex& block_index : all_tsdf_blocks) {
      mesh_layer_->allocateMeshPtrByIndex(block_index);
    }

    ThreadSafeIndex index_getter(all_tsdf_blocks.size());

    std::list<std::thread> integration_threads;
    for (size_t i = 0; i < config_.integrator_threads; ++i) {
      integration_threads.emplace_back(
          &MeshIntegrator::generateMeshBlocksFunction, this, all_tsdf_blocks,
          clear_updated_flag, &index_getter);
    }

    for (std::thread& thread : integration_threads) {
      thread.join();
    }
    return true;
  }

  void generateMeshBlocksFunction(const BlockIndexList& all_tsdf_blocks,
                                  bool clear_updated_flag,
                                  ThreadSafeIndex* index_getter) {
    DCHECK(index_getter != nullptr);

    size_t list_idx;
    while (index_getter->getNextIndex(&list_idx)) {
      const BlockIndex& block_idx = all_tsdf_blocks[list_idx];
      typename Block<TsdfVoxel>::Ptr tsdf_block =
          sdf_layer_mutable_->getBlockPtrByIndex(block_idx);
      typename Block<LabelVoxel>::Ptr label_block =
          label_layer_mutable_->getBlockPtrByIndex(block_idx);

      updateMeshForBlock(block_idx);
      if (clear_updated_flag) {
        tsdf_block->updated() = false;
        label_block->updated() = false;
      }
    }
  }

  Color getColorFromLabel(const Label& label) {
    Color color;
    CHECK_NE(label, 0u);
    auto label_color_map_it = label_color_map_.find(label);

    if (label_color_map_it != label_color_map_.end()) {
      color = label_color_map_it->second;
    } else {
      color = randomColor();
      label_color_map_.insert(std::pair<Label, Color>(label, color));
    }

    // TODO(grinvalm): fix or remove the flushing color visualization below.
    // if (!label_age_map_ptr_->empty()) {
    //   std::map<Label, int>::const_iterator label_age_pair_it;
    //   label_age_pair_it = label_age_map_ptr_->find(label);
    //   if (label_age_pair_it != label_age_map_ptr_->end()) {
    //     // color.r = color.r - label_age_pair_it->second * 10;
    //     // color.g = color.g - label_age_pair_it->second * 10;
    //     // color.b = color.b - label_age_pair_it->second * 10;
    //     if (label_age_pair_it->second < 4) {
    //       color.g = 0;
    //       // color.a = 255;
    //     } else if (label_age_pair_it->second > 3) {
    //       color.r = 0;
    //       color.g = 255;
    //       color.b = 0;
    //     } else {
    //       // int shade_of_gray = (color.r + color.g + color.b) % 256;
    //       int shade_of_gray = (color.r + color.g + color.b) % 256;
    //       color.r = shade_of_gray;
    //       color.g = shade_of_gray;
    //       color.b = shade_of_gray;
    //     }
    //   } else {
    //     int shade_of_gray = (color.r + color.g + color.b) % 256;
    //     color.r = shade_of_gray;
    //     color.g = shade_of_gray;
    //     color.b = shade_of_gray;
    //   }
    // } else {
    //   int shade_of_gray = (color.r + color.g + color.b) % 256;
    //   color.r = shade_of_gray;
    //   color.g = shade_of_gray;
    //   color.b = shade_of_gray;
    // }
    return color;
  }

  virtual void updateMeshForBlock(const BlockIndex& block_index) {
    Mesh::Ptr mesh = mesh_layer_->getMeshPtrByIndex(block_index);
    mesh->clear();
    // This block should already exist, otherwise it makes no sense to update
    // the mesh for it. ;)
    Block<TsdfVoxel>::ConstPtr tsdf_block =
        sdf_layer_const_->getBlockPtrByIndex(block_index);
    Block<LabelVoxel>::ConstPtr label_block =
        label_layer_const_->getBlockPtrByIndex(block_index);

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
      if (label_block) {
        updateMeshColor(*label_block, mesh.get());
      }
    }

    mesh->updated = true;
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
            label_layer_const_->getBlockPtrByCoordinates(vertex);
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
  // Having both a const and a mutable pointer to the layer allows this
  // integrator to work both with a const layer (in case you don't want to clear
  // the updated flag) and mutable layer (in case you do want to clear the
  // updated flag).
  Layer<LabelVoxel>* label_layer_mutable_;
  const Layer<LabelVoxel>* label_layer_const_;

  // Flag to choose whether the mesh color
  // encodes the labels or the label confidences.
  // By default the mesh encodes the labels.
  bool visualize_confidence;

  std::map<Label, Color> label_color_map_;
  const std::map<Label, int>* label_age_map_ptr_;
};

}  // namespace voxblox

#endif  // GLOBAL_SEGMENT_MAP_LABEL_TSDF_MESH_INTEGRATOR_H_
