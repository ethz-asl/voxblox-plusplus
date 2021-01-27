// NOTE: Code adapted from open_chisel:
// github.com/personalrobotics/OpenChisel/

// The MIT License (MIT) Copyright (c)
// 2014 Matthew Klingensmith and Ivan Dryanovski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "global_segment_map/meshing/label_tsdf_mesh_integrator.h"

#include "voxblox/core/voxel.h"

namespace voxblox {
MeshLabelIntegrator::MeshLabelIntegrator(
    const MeshIntegratorConfig& config,
    const MeshLabelIntegrator::LabelTsdfConfig& label_tsdf_config,
    LabelTsdfMap* map, MeshLayer* mesh_layer, bool* remesh)
    : MeshIntegrator(config, CHECK_NOTNULL(map)->getTsdfLayerPtr(), mesh_layer),
      label_tsdf_config_(label_tsdf_config),
      label_layer_mutable_ptr_(CHECK_NOTNULL(map->getLabelLayerPtr())),
      label_layer_const_ptr_(CHECK_NOTNULL(map->getLabelLayerPtr())),
      semantic_instance_label_fusion_ptr_(
          map->getSemanticInstanceLabelFusionPtr()),
      label_color_map_(),
      instance_color_map_(),
      semantic_color_map_(
          SemanticColorMap::create(label_tsdf_config.class_task)),
      remesh_ptr_(remesh) {
  if (remesh_ptr_ == nullptr) {
    remesh_ptr_ = &remesh_;
  }
}

MeshLabelIntegrator::MeshLabelIntegrator(
    const MeshIntegratorConfig& config,
    const MeshLabelIntegrator::LabelTsdfConfig& label_tsdf_config,
    const LabelTsdfMap& map, MeshLayer* mesh_layer, bool* remesh)
    : MeshIntegrator(config, map.getTsdfLayer(), mesh_layer),
      label_tsdf_config_(label_tsdf_config),
      label_layer_mutable_ptr_(nullptr),
      label_layer_const_ptr_(CHECK_NOTNULL(&map.getLabelLayer())),
      semantic_instance_label_fusion_ptr_(
          &map.getSemanticInstanceLabelFusion()),
      label_color_map_(),
      instance_color_map_(),
      semantic_color_map_(
          SemanticColorMap::create(label_tsdf_config.class_task)),
      remesh_ptr_(remesh) {
  if (remesh_ptr_ == nullptr) {
    remesh_ptr_ = &remesh_;
  }
}

MeshLabelIntegrator::MeshLabelIntegrator(
    const MeshIntegratorConfig& config,
    const MeshLabelIntegrator::LabelTsdfConfig& label_tsdf_config,
    const Layer<TsdfVoxel>& tsdf_layer, const Layer<LabelVoxel>& label_layer,
    MeshLayer* mesh_layer)
    : MeshIntegrator(config, tsdf_layer, mesh_layer),
      label_tsdf_config_(label_tsdf_config),
      label_layer_mutable_ptr_(nullptr),
      label_layer_const_ptr_(&label_layer),
      semantic_instance_label_fusion_ptr_(nullptr),
      label_color_map_(),
      instance_color_map_(),
      semantic_color_map_(
          SemanticColorMap::create(label_tsdf_config.class_task)) {}

bool MeshLabelIntegrator::generateMesh(bool only_mesh_updated_blocks,
                                       bool clear_updated_flag) {
  CHECK(!clear_updated_flag || ((sdf_layer_mutable_ != nullptr) &&
                                (label_layer_mutable_ptr_ != nullptr)))
      << "If you would like to modify the updated flag in the blocks, please "
      << "use the constructor that provides a non-const link to the sdf and "
         "label layers!";

  BlockIndexList all_tsdf_blocks;

  if (only_mesh_updated_blocks) {
    BlockIndexList all_label_blocks;
    sdf_layer_const_->getAllUpdatedBlocks(&all_tsdf_blocks);
    label_layer_mutable_ptr_->getAllUpdatedBlocks(&all_label_blocks);
    if (all_tsdf_blocks.size() == 0u && all_label_blocks.size() == 0u) {
      return false;
    }

    all_tsdf_blocks.insert(all_tsdf_blocks.end(), all_label_blocks.begin(),
                           all_label_blocks.end());
  } else {
    sdf_layer_const_->getAllAllocatedBlocks(&all_tsdf_blocks);
  }

  // Allocate all the mesh memory
  for (const BlockIndex& block_index : all_tsdf_blocks) {
    mesh_layer_->allocateMeshPtrByIndex(block_index);
  }

  std::unique_ptr<ThreadSafeIndex> index_getter(
      new MixedThreadSafeIndex(all_tsdf_blocks.size()));

  std::list<std::thread> integration_threads;
  for (size_t i = 0u; i < config_.integrator_threads; ++i) {
    integration_threads.emplace_back(
        &MeshLabelIntegrator::generateMeshBlocksFunction, this,
        std::cref(all_tsdf_blocks), clear_updated_flag, index_getter.get());
  }

  for (std::thread& thread : integration_threads) {
    thread.join();
  }

  return true;
}

void MeshLabelIntegrator::generateMeshBlocksFunction(
    const BlockIndexList& all_tsdf_blocks, bool clear_updated_flag,
    ThreadSafeIndex* index_getter) {
  CHECK(!clear_updated_flag || (sdf_layer_mutable_ != nullptr) ||
        (label_layer_mutable_ptr_ != nullptr))
      << "If you would like to modify the updated flag in the blocks, please "
      << "use the constructor that provides a non-const link to the sdf and "
         "label layers!";
  CHECK_NOTNULL(index_getter);

  size_t list_idx;
  while (index_getter->getNextIndex(&list_idx)) {
    const BlockIndex& block_idx = all_tsdf_blocks[list_idx];
    updateMeshForBlock(block_idx);
    if (clear_updated_flag) {
      typename Block<TsdfVoxel>::Ptr tsdf_block =
          sdf_layer_mutable_->getBlockPtrByIndex(block_idx);
      typename Block<LabelVoxel>::Ptr label_block =
          label_layer_mutable_ptr_->getBlockPtrByIndex(block_idx);

      tsdf_block->updated() = false;
      label_block->updated() = false;
    }
  }
}

// TODO(margaritaG): handle this remeshing!!
InstanceLabel MeshLabelIntegrator::getInstanceLabel(const Label& label) {
  float kFramesCountThresholdFactor = 0.1f;

  InstanceLabel instance_label =
      semantic_instance_label_fusion_ptr_->getInstanceLabel(
          label, kFramesCountThresholdFactor);
  std::map<Label, InstanceLabel>::iterator prev_instance_it;
  {
    std::shared_lock<std::shared_timed_mutex> readerLock(
        label_instance_map_mutex_);
    prev_instance_it = label_instance_map_.find(label);
  }

  if (prev_instance_it != label_instance_map_.end()) {
    if (prev_instance_it->second != instance_label) {
      *remesh_ptr_ = true;
    }
  }
  std::lock_guard<std::shared_timed_mutex> writerLock(
      label_instance_map_mutex_);
  label_instance_map_[label] = instance_label;
  return instance_label;
}

void MeshLabelIntegrator::updateMeshForBlock(const BlockIndex& block_index) {
  Mesh::Ptr mesh_block = mesh_layer_->getMeshPtrByIndex(block_index);
  mesh_block->clear();
  // This block should already exist, otherwise it makes no sense to update
  // the mesh for it. ;)
  Block<TsdfVoxel>::ConstPtr tsdf_block =
      sdf_layer_const_->getBlockPtrByIndex(block_index);
  Block<LabelVoxel>::ConstPtr label_block =
      label_layer_const_ptr_->getBlockPtrByIndex(block_index);

  if (!tsdf_block && !label_block) {
    LOG(ERROR) << "Trying to mesh a non-existent block at index: "
               << block_index.transpose();
    return;
  }
  // TODO(margaritaG): this is actually possible because with voxel carving we
  // do not allocate labels along the ray, just nearby surfaces.
  // } else if (!(tsdf_block && label_block)) {
  //   LOG(FATAL) << "Block allocation differs between the two layers.";
  // }
  extractBlockMesh(tsdf_block, mesh_block);
  // Update colors if needed.
  if (config_.use_color) {
    updateMeshBlockColor(tsdf_block, label_block, mesh_block.get());
  }

  mesh_block->updated = true;
}

void MeshLabelIntegrator::updateMeshBlockColor(
    Block<TsdfVoxel>::ConstPtr tsdf_block,
    Block<LabelVoxel>::ConstPtr label_block, Mesh* mesh_block) {
  CHECK_NOTNULL(mesh_block);
  switch (label_tsdf_config_.color_scheme) {
    case kColor:
      updateMeshColor(*tsdf_block, mesh_block);
      break;
    case kNormals:
      updateMeshColor(*tsdf_block, mesh_block);
      break;
    default:
      if (!label_block) {
        LOG(FATAL)
            << "Trying to color a mesh using a non-existent label block.";
      }
      updateMeshColor(*label_block, mesh_block);
  }
}

void MeshLabelIntegrator::updateMeshColor(const Block<LabelVoxel>& label_block,
                                          Mesh* mesh) {
  CHECK_NOTNULL(mesh);

  mesh->colors.clear();
  mesh->colors.resize(mesh->indices.size());

  // Use nearest-neighbor search.
  for (size_t i = 0u; i < mesh->vertices.size(); ++i) {
    const Point& vertex = mesh->vertices[i];
    VoxelIndex voxel_index =
        label_block.computeVoxelIndexFromCoordinates(vertex);
    if (label_block.isValidVoxelIndex(voxel_index)) {
      const LabelVoxel& voxel = label_block.getVoxelByVoxelIndex(voxel_index);
      switch (label_tsdf_config_.color_scheme) {
        case kLabelConfidence: {
          utils::getColorFromLabelConfidence(
              voxel, label_tsdf_config_.max_confidence, &(mesh->colors[i]));
        } break;
        case kLabel: {
          label_color_map_.getColor(voxel.label, &(mesh->colors[i]));
        } break;
        case kSemantic: {
          SemanticLabel semantic_label = 0u;
          InstanceLabel instance_label = getInstanceLabel(voxel.label);
          if (instance_label != BackgroundLabel) {
            semantic_label =
                semantic_instance_label_fusion_ptr_->getSemanticLabel(
                    voxel.label);
          }
          semantic_color_map_.getColor(semantic_label, &(mesh->colors[i]));
        } break;
        case kInstance: {
          InstanceLabel instance_label = getInstanceLabel(voxel.label);
          instance_color_map_.getColor(instance_label, &(mesh->colors[i]));
        } break;
        case kMerged: {
          InstanceLabel instance_label = getInstanceLabel(voxel.label);
          if (instance_label == BackgroundLabel) {
            label_color_map_.getColor(voxel.label, &(mesh->colors[i]));
          } else {
            instance_color_map_.getColor(instance_label, &(mesh->colors[i]));
          }
        } break;
        default: {
          LOG(FATAL) << "Unknown mesh color scheme: "
                     << label_tsdf_config_.color_scheme;
        }
      }
    } else {
      const typename Block<LabelVoxel>::ConstPtr neighbor_block =
          label_layer_const_ptr_->getBlockPtrByCoordinates(vertex);
      const LabelVoxel& voxel = neighbor_block->getVoxelByCoordinates(vertex);
      switch (label_tsdf_config_.color_scheme) {
        case kLabel: {
          label_color_map_.getColor(voxel.label, &(mesh->colors[i]));
        } break;
        case kLabelConfidence: {
          utils::getColorFromLabelConfidence(
              voxel, label_tsdf_config_.max_confidence, &(mesh->colors[i]));
        } break;
        case kSemantic: {
          SemanticLabel semantic_label = 0u;
          InstanceLabel instance_label = getInstanceLabel(voxel.label);
          if (instance_label != BackgroundLabel) {
            semantic_label =
                semantic_instance_label_fusion_ptr_->getSemanticLabel(
                    voxel.label);
          }
          semantic_color_map_.getColor(semantic_label, &(mesh->colors[i]));
        } break;
        case kInstance: {
          InstanceLabel instance_label = getInstanceLabel(voxel.label);
          instance_color_map_.getColor(instance_label, &(mesh->colors[i]));
        } break;
        case kMerged: {
          InstanceLabel instance_label = getInstanceLabel(voxel.label);
          if (instance_label == BackgroundLabel) {
            label_color_map_.getColor(voxel.label, &(mesh->colors[i]));
          } else {
            instance_color_map_.getColor(instance_label, &(mesh->colors[i]));
          }
        } break;
        default: {
          LOG(FATAL) << "Unknown mesh color scheme: "
                     << label_tsdf_config_.color_scheme;
        }
      }
    }
  }
}

void MeshLabelIntegrator::updateMeshColor(const Block<TsdfVoxel>& tsdf_block,
                                          Mesh* mesh) {
  CHECK_NOTNULL(mesh);

  mesh->colors.clear();
  mesh->colors.resize(mesh->indices.size());

  // Use nearest-neighbor search.
  for (size_t i = 0u; i < mesh->vertices.size(); i++) {
    const Point& vertex = mesh->vertices[i];
    VoxelIndex voxel_index =
        tsdf_block.computeVoxelIndexFromCoordinates(vertex);
    if (tsdf_block.isValidVoxelIndex(voxel_index)) {
      const TsdfVoxel& voxel = tsdf_block.getVoxelByVoxelIndex(voxel_index);
      switch (label_tsdf_config_.color_scheme) {
        case kColor: {
          utils::getColorIfValid(voxel, config_.min_weight, &(mesh->colors[i]));
        } break;
        case kNormals: {
          utils::getColorFromNormals(mesh->normals[i], &(mesh->colors[i]));
        } break;
        default: {
          LOG(FATAL) << "Unknown mesh color scheme: "
                     << label_tsdf_config_.color_scheme;
        }
      }
    } else {
      const typename Block<TsdfVoxel>::ConstPtr neighbor_block =
          sdf_layer_const_->getBlockPtrByCoordinates(vertex);
      const TsdfVoxel& voxel = neighbor_block->getVoxelByCoordinates(vertex);
      switch (label_tsdf_config_.color_scheme) {
        case kColor: {
          utils::getColorIfValid(voxel, config_.min_weight, &(mesh->colors[i]));
        } break;
        case kNormals: {
          utils::getColorFromNormals(mesh->normals[i], &(mesh->colors[i]));
        } break;
        default: {
          LOG(FATAL) << "Unknown mesh color scheme: "
                     << label_tsdf_config_.color_scheme;
        }
      }
    }
  }
}

}  // namespace voxblox
