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

class MeshLabelIntegrator : public MeshIntegrator {
 public:
  MeshLabelIntegrator(const Config& config,
                      Layer<TsdfVoxel>* tsdf_layer,
                      Layer<LabelVoxel>* label_layer,
                      MeshLayer* mesh_layer,
                      bool visualize_confidence = false)
      : MeshIntegrator(config, tsdf_layer, mesh_layer),
        label_layer_(CHECK_NOTNULL(label_layer)),
        visualize_confidence(visualize_confidence) {}

  void generateMeshForUpdatedBlocks(bool clear_updated_flag, bool* updated) {
    // Only update parts of the mesh for blocks that have updated.
    // clear_updated_flag decides whether to reset 'updated' after updating the
    // mesh.
    BlockIndexList all_tsdf_blocks;
    tsdf_layer_->getAllAllocatedBlocks(&all_tsdf_blocks);

    for (const BlockIndex& block_index : all_tsdf_blocks) {
      Block<TsdfVoxel>::Ptr tsdf_block =
          tsdf_layer_->getBlockPtrByIndex(block_index);
      Block<LabelVoxel>::Ptr label_block =
          label_layer_->getBlockPtrByIndex(block_index);
      if (tsdf_block->updated() || label_block->updated()) {
        *updated = true;
        updateMeshForBlock(block_index);
        if (clear_updated_flag) {
          tsdf_block->updated() = false;
          label_block->updated() = false;
        }
      }
    }
  }

  virtual void updateMeshForBlock(const BlockIndex& block_index) {
    Mesh::Ptr mesh = mesh_layer_->allocateMeshPtrByIndex(block_index);
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

    if (config_.compute_normals) {
      computeMeshNormals(*tsdf_block, mesh.get());
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


  void updateMeshColor(const Block<LabelVoxel>& label_block, Mesh* mesh) {
    CHECK_NOTNULL(mesh);

    mesh->colors.clear();
    mesh->colors.resize(mesh->indices.size());

    // Use nearest-neighbor search.
    for (size_t i = 0u; i < mesh->vertices.size(); i++) {
      const Point& vertex = mesh->vertices[i];
      VoxelIndex voxel_index =
          label_block.computeVoxelIndexFromCoordinates(vertex);
      const Point voxel_center =
          label_block.computeCoordinatesFromVoxelIndex(voxel_index);

      // Should be within half a voxel of the voxel center in all dimensions, or
      // it belongs in the other one.
      const Point dist_from_center = vertex - voxel_center;
      for (unsigned int j = 0u; j < 3u; ++j) {
        if (std::abs(dist_from_center(j)) > voxel_size_ / 2.0) {
          voxel_index(j) += signum(dist_from_center(j));
        }
      }

      if (label_block.isValidVoxelIndex(voxel_index)) {
        Color color;
        if (visualize_confidence) {
          color = rainbowColorMap(
              label_block.getVoxelByVoxelIndex(voxel_index).label_confidence / 100);
        } else {
          color = getColorFromLabel(
              label_block.getVoxelByVoxelIndex(voxel_index).label);
        }
        mesh->colors[i] = color;
      } else {
        // Get the nearest block.
        const Block<LabelVoxel>::ConstPtr neighbor_block =
            label_layer_->getBlockPtrByCoordinates(vertex);
        if (neighbor_block) {
          Color color;
          if (visualize_confidence) {
            color = rainbowColorMap(
                neighbor_block->getVoxelByCoordinates(vertex).label_confidence / 100);
          } else {
            color = getColorFromLabel(
                neighbor_block->getVoxelByCoordinates(vertex).label);
          }
          mesh->colors[i] = color;
        }
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
