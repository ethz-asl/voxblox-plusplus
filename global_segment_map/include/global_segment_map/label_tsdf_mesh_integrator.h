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
#include "global_segment_map/utils/semantic_color_map.h"

namespace voxblox {

class MeshLabelIntegrator : public MeshIntegrator<TsdfVoxel> {
 public:
  enum ColorScheme {
    LabelColor = 1,
    SemanticColor = 2,
    InstanceColor = 3,
    GeometricInstanceColor = 4,
    ConfidenceColor = 5
  };

  MeshLabelIntegrator(
      const MeshIntegratorConfig& config, Layer<TsdfVoxel>* tsdf_layer,
      Layer<LabelVoxel>* label_layer, MeshLayer* mesh_layer,
      std::set<SemanticLabel>& all_semantic_labels,
      const utils::InstanceLabelFusion* instance_label_fusion = nullptr,
      const utils::SemanticLabelFusion* semantic_label_fusion = nullptr,
      ColorScheme color_scheme = LabelColor, bool* remesh = nullptr,
      SemanticColorMap::SemanticColor semantic_color_mode =
          SemanticColorMap::kCoco)
      : MeshIntegrator(config, tsdf_layer, mesh_layer),
        label_layer_mutable_ptr_(CHECK_NOTNULL(label_layer)),
        label_layer_const_ptr_(CHECK_NOTNULL(label_layer)),
        instance_label_fusion_ptr_(instance_label_fusion),
        semantic_label_fusion_ptr_(semantic_label_fusion),
        all_semantic_labels_ptr_(&all_semantic_labels),
        color_scheme_(color_scheme),
        color_map_(SemanticColorMap::create(semantic_color_mode)),
        remesh_ptr_(remesh) {
    if (remesh_ptr_ == nullptr) {
      remesh_ptr_ = &remesh_;
    }
  }

  MeshLabelIntegrator(
      const MeshIntegratorConfig& config, const Layer<TsdfVoxel>& tsdf_layer,
      const Layer<LabelVoxel>& label_layer, MeshLayer* mesh_layer,
      std::set<SemanticLabel>& all_semantic_labels,
      const utils::InstanceLabelFusion* instance_label_fusion = nullptr,
      const utils::SemanticLabelFusion* semantic_label_fusion = nullptr,
      ColorScheme color_scheme = LabelColor, bool* remesh = nullptr,
      SemanticColorMap::SemanticColor semantic_color_mode =
          SemanticColorMap::kCoco)
      : MeshIntegrator(config, tsdf_layer, mesh_layer),
        label_layer_mutable_ptr_(nullptr),
        label_layer_const_ptr_(CHECK_NOTNULL(&label_layer)),
        instance_label_fusion_ptr_(instance_label_fusion),
        semantic_label_fusion_ptr_(semantic_label_fusion),
        all_semantic_labels_ptr_(&all_semantic_labels),
        color_scheme_(color_scheme),
        color_map_(SemanticColorMap::create(semantic_color_mode)),
        remesh_ptr_(remesh) {
    if (remesh_ptr_ == nullptr) {
      remesh_ptr_ = &remesh_;
    }
  }

  // Generates mesh for the tsdf layer.
  bool generateMesh(bool only_mesh_updated_blocks, bool clear_updated_flag) {
    CHECK(!clear_updated_flag || ((sdf_layer_mutable_ != nullptr) &&
                                  (label_layer_mutable_ptr_ != nullptr)))
        << "If you would like to modify the updated flag in the blocks, please "
        << "use the constructor that provides a non-const link to the sdf and "
           "label layers!";

    BlockIndexList all_tsdf_blocks;
    // BlockIndexList all_label_blocks;
    if (only_mesh_updated_blocks) {
      sdf_layer_const_->getAllUpdatedBlocks(&all_tsdf_blocks);
      // TODO(grinvalm) unique union of block indices here.
      // label_layer_mutable_ptr_->getAllUpdatedBlocks(&all_label_blocks);
      if (all_tsdf_blocks.size() == 0u) {
        return false;
      }
      // all_tsdf_blocks.insert(all_tsdf_blocks.end(), all_label_blocks.begin(),
      //                        all_label_blocks.end());
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
          &MeshLabelIntegrator::generateMeshBlocksFunction, this,
          all_tsdf_blocks, clear_updated_flag, &index_getter);
    }

    for (std::thread& thread : integration_threads) {
      thread.join();
    }
    return true;
  }

  void generateMeshBlocksFunction(const BlockIndexList& all_tsdf_blocks,
                                  bool clear_updated_flag,
                                  ThreadSafeIndex* index_getter) {
    CHECK(!clear_updated_flag || (sdf_layer_mutable_ != nullptr) ||
          (label_layer_mutable_ptr_ != nullptr))
        << "If you would like to modify the updated flag in the blocks, please "
        << "use the constructor that provides a non-const link to the sdf and "
           "label layers!";
    DCHECK(index_getter != nullptr);

    size_t list_idx;
    while (index_getter->getNextIndex(&list_idx)) {
      const BlockIndex& block_idx = all_tsdf_blocks[list_idx];
      typename Block<TsdfVoxel>::Ptr tsdf_block =
          sdf_layer_mutable_->getBlockPtrByIndex(block_idx);
      typename Block<LabelVoxel>::Ptr label_block =
          label_layer_mutable_ptr_->getBlockPtrByIndex(block_idx);

      updateMeshForBlock(block_idx);
      if (clear_updated_flag) {
        typename Block<TsdfVoxel>::Ptr tsdf_block =
            sdf_layer_mutable_->getBlockPtrByIndex(block_idx);
        typename Block<LabelVoxel>::Ptr label_block =
            label_layer_mutable_ptr_->getBlockPtrByIndex(block_idx);

        tsdf_block->updated() = false;
        // label_block->updated() = false;
      }
    }
  }

  // TODO(margaritaG): handle this remeshing!!

  // SemanticLabel getLabelInstance(const Label& label) {
  //   SemanticLabel instance_label = 0u;
  //   int max_count = 0;
  //   auto label_it = label_instance_count_ptr_->find(label);
  //   if (label_it != label_instance_count_ptr_->end()) {
  //     for (auto const& instance_count : label_it->second) {
  //       if (instance_count.second > max_count && instance_count.first != 0u)
  //       {
  //         int frames_count = 0;
  //         auto label_count_it = label_frames_count_ptr_->find(label);
  //         if (label_count_it != label_frames_count_ptr_->end()) {
  //           frames_count = label_count_it->second;
  //         }
  //         if (instance_count.second >
  //             0.1f * (float)(frames_count - instance_count.second)) {
  //           instance_label = instance_count.first;
  //           max_count = instance_count.second;
  //         }
  //       }
  //     }
  //   } else {
  //     // LOG(ERROR) << "No semantic class for label?";
  //   }
  //   auto prev_instance_it = label_instance_map_.find(label);
  //   if (prev_instance_it != label_instance_map_.end()) {
  //     if (prev_instance_it->second != instance_label) {
  //       *remesh_ptr_ = true;
  //     }
  //   }
  //   label_instance_map_[label] = instance_label;
  //   return instance_label;
  // }

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
    return color;
  }

  Color getColorFromInstanceLabel(const SemanticLabel& instance_label) {
    Color color;
    auto instance_color_map_it = instance_color_map_.find(instance_label);

    if (instance_color_map_it != instance_color_map_.end()) {
      color = instance_color_map_it->second;
    } else {
      color = randomColor();
      if (instance_label == 0u) {
        color.r = 200;
        color.g = 200;
        color.b = 200;
      }
      instance_color_map_.insert(
          std::pair<SemanticLabel, Color>(instance_label, color));
    }

    return color;
  }

  Color getColorFromSemanticLabel(const SemanticLabel& semantic_label) {
    return color_map_.getColor(semantic_label);
  }

  virtual void updateMeshForBlock(const BlockIndex& block_index) {
    Mesh::Ptr mesh = mesh_layer_->getMeshPtrByIndex(block_index);
    mesh->clear();
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
    // } else if (!(tsdf_block && label_block)) {
    //   LOG(FATAL) << "Block allocation differs between the two layers.";
    // }

    extractBlockMesh(tsdf_block, mesh);
    // Update colors if needed.
    // updateMeshColor(*tsdf_block, mesh.get());
    if (config_.use_color) {
      if (label_block) {
        updateMeshColor(*label_block, mesh.get());
      } else {
        // updateMeshColor(*tsdf_block, mesh.get());
        // LOG(FATAL) << "NO LABEL VOXELS TO COLOR " << mesh->indices.size()
        //            << " mesh indices.";

        // mesh->colors.clear();
        // mesh->colors.resize(mesh->indices.size());
        //
        // // Use nearest-neighbor search.
        // for (size_t i = 0; i < mesh->vertices.size(); ++i) {
        //   Color color;
        //   color.r = 255;
        //   color.g = 0;
        //   color.b = 0;
        //   mesh->colors[i] = color;
        // }
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
        if (color_scheme_ == ConfidenceColor) {
          // Scale values to range (0.0, 1.0).
          constexpr float expected_max_confidence = 100.0f;
          color =
              rainbowColorMap(voxel.label_confidence / expected_max_confidence);
        } else if (color_scheme_ == LabelColor) {
          color = getColorFromLabel(voxel.label);
        } else if (color_scheme_ == SemanticColor) {
          InstanceLabel instance_label =
              instance_label_fusion_ptr_->getLabelInstance(voxel.label);
          SemanticLabel semantic_label = 0u;
          if (instance_label != 0u) {
            semantic_label =
                semantic_label_fusion_ptr_->getSemanticLabel(voxel.label);
            all_semantic_labels_ptr_->insert(semantic_label);
          }
          // color = getColorFromInstanceLabel(semantic_label);
          color = getColorFromSemanticLabel(semantic_label);
        } else if (color_scheme_ == InstanceColor) {
          InstanceLabel instance_label =
              instance_label_fusion_ptr_->getLabelInstance(voxel.label);
          if (instance_label == 0u) {
            // LOG(ERROR) << "Label 0";
          } else {
            // LOG(ERROR) << unsigned(instance_label);
          }
          color = getColorFromInstanceLabel(instance_label);
        } else if (color_scheme_ == GeometricInstanceColor) {
          InstanceLabel instance_label =
              instance_label_fusion_ptr_->getLabelInstance(voxel.label);
          if (instance_label == 0u) {
            color = getColorFromLabel(voxel.label);
          } else {
            color = getColorFromInstanceLabel(instance_label);
          }
        }
        mesh->colors[i] = color;
      } else {
        const typename Block<LabelVoxel>::ConstPtr neighbor_block =
            label_layer_const_ptr_->getBlockPtrByCoordinates(vertex);
        const LabelVoxel& voxel = neighbor_block->getVoxelByCoordinates(vertex);
        Color color;
        if (color_scheme_ == ConfidenceColor) {
          // Scale values to range (0.0, 1.0).
          constexpr float expected_max_confidence = 100.0f;
          color =
              rainbowColorMap(voxel.label_confidence / expected_max_confidence);
        } else if (color_scheme_ == LabelColor) {
          color = getColorFromLabel(voxel.label);
        } else if (color_scheme_ == SemanticColor) {
          InstanceLabel instance_label =
              instance_label_fusion_ptr_->getLabelInstance(voxel.label);
          SemanticLabel semantic_label = 0u;
          if (instance_label != 0u) {
            semantic_label =
                semantic_label_fusion_ptr_->getSemanticLabel(voxel.label);
            all_semantic_labels_ptr_->insert(semantic_label);
          }
          // color = getColorFromInstanceLabel(semantic_label);
          color = getColorFromSemanticLabel(semantic_label);
        } else if (color_scheme_ == InstanceColor) {
          InstanceLabel instance_label =
              instance_label_fusion_ptr_->getLabelInstance(voxel.label);
          color = getColorFromInstanceLabel(instance_label);
        } else if (color_scheme_ == GeometricInstanceColor) {
          InstanceLabel instance_label =
              instance_label_fusion_ptr_->getLabelInstance(voxel.label);
          if (instance_label == 0u) {
            color = getColorFromLabel(voxel.label);
          } else {
            color = getColorFromInstanceLabel(instance_label);
          }
        }
        mesh->colors[i] = color;
      }
    }
  }

 protected:
  // Having both a const and a mutable pointer to the layer allows this
  // integrator to work both with a const layer (in case you don't want to
  // clear the updated flag) and mutable layer (in case you do want to clear
  // the updated flag).
  Layer<LabelVoxel>* label_layer_mutable_ptr_;
  const Layer<LabelVoxel>* label_layer_const_ptr_;

  // Color scheme to use for the mesh color.
  // By default the mesh encodes the labels.
  ColorScheme color_scheme_;

  std::map<Label, Color> label_color_map_;
  std::map<SemanticLabel, Color> instance_color_map_;

  const utils::InstanceLabelFusion* instance_label_fusion_ptr_;
  const utils::SemanticLabelFusion* semantic_label_fusion_ptr_;

  // const std::map<Label, std::map<SemanticLabel, int>>*
  // label_class_count_ptr_; const std::map<Label, std::map<SemanticLabel,
  // int>>* label_instance_count_ptr_;
  // const std::map<Label, int>* label_frames_count_ptr_;
  std::set<SemanticLabel>* all_semantic_labels_ptr_;
  bool* remesh_ptr_;
  // This parameter is used if no valid remesh_ptr is provided to the class at
  // construction time.
  bool remesh_ = false;
  std::map<Label, SemanticLabel> label_instance_map_;

  SemanticColorMap color_map_;
};

}  // namespace voxblox

#endif  // GLOBAL_SEGMENT_MAP_LABEL_TSDF_MESH_INTEGRATOR_H_
