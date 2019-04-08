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
#include "global_segment_map/utils/color_map.h"

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
      const std::map<Label, int>& label_age_map = {},
      ColorScheme color_scheme = LabelColor, bool* remesh = nullptr,
      SemanticColorMap::SemanticColor semantic_color_mode =
          SemanticColorMap::kNyu13)
      : MeshIntegrator(config, tsdf_layer, mesh_layer),
        label_layer_mutable_ptr_(CHECK_NOTNULL(label_layer)),
        label_layer_const_ptr_(CHECK_NOTNULL(label_layer)),
        instance_label_fusion_ptr_(instance_label_fusion),
        semantic_label_fusion_ptr_(semantic_label_fusion),
        all_semantic_labels_ptr_(&all_semantic_labels),
        label_age_map_ptr_(&label_age_map),
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
      const std::map<Label, int>& label_age_map = {},
      ColorScheme color_scheme = LabelColor, bool* remesh = nullptr,
      SemanticColorMap::SemanticColor semantic_color_mode =
          SemanticColorMap::kNyu13)
      : MeshIntegrator(config, tsdf_layer, mesh_layer),
        label_layer_mutable_ptr_(nullptr),
        label_layer_const_ptr_(CHECK_NOTNULL(&label_layer)),
        instance_label_fusion_ptr_(instance_label_fusion),
        semantic_label_fusion_ptr_(semantic_label_fusion),
        all_semantic_labels_ptr_(&all_semantic_labels),
        label_age_map_ptr_(&label_age_map),
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

    // // Allocate all the mesh memory
    // for (const BlockIndex& block_index : all_label_blocks) {
    //   mesh_layer_->allocateMeshPtrByIndex(block_index);
    // }
    //
    // ThreadSafeIndex index_getter_labels(all_label_blocks.size());
    //
    // std::list<std::thread> integration_threads_labels;
    // for (size_t i = 0; i < config_.integrator_threads; ++i) {
    //   integration_threads_labels.emplace_back(
    //       &MeshLabelIntegrator::generateMeshBlocksFunction, this,
    //       all_label_blocks, clear_updated_flag, &index_getter_labels);
    // }
    //
    // for (std::thread& thread : integration_threads_labels) {
    //   thread.join();
    // }
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
    color_map_.getColor(semantic_label);

    std::vector<std::array<float, 3>> coco_color_code{
        {200, 200, 200}, {128, 0, 0},    {0, 128, 0},    {128, 128, 0},
        {0, 0, 128},     {128, 0, 128},  {0, 128, 128},  {128, 128, 128},
        {64, 0, 0},      {192, 0, 0},    {64, 128, 0},   {192, 128, 0},
        {64, 0, 128},    {192, 0, 128},  {64, 128, 128}, {192, 128, 128},
        {0, 64, 0},      {128, 64, 0},   {0, 192, 0},    {128, 192, 0},
        {0, 64, 128},    {128, 64, 128}, {0, 192, 128},  {128, 192, 128},
        {64, 64, 0},     {192, 64, 0},   {64, 192, 0},   {192, 192, 0},
        {64, 64, 128},   {192, 64, 128}, {64, 192, 128}, {192, 192, 128},
        {0, 0, 64},      {128, 0, 64},   {0, 128, 64},   {128, 128, 64},
        {0, 0, 192},     {128, 0, 192},  {0, 128, 192},  {128, 128, 192},
        {64, 0, 64},     {192, 0, 64},   {64, 128, 64},  {192, 128, 64},
        {64, 0, 192},    {192, 0, 192},  {64, 128, 192}, {192, 128, 192},
        {0, 64, 64},     {128, 64, 64},  {0, 192, 64},   {128, 192, 64},
        {0, 64, 192},    {128, 64, 192}, {0, 192, 192},  {128, 192, 192},
        {64, 64, 64},    {192, 64, 64},  {64, 192, 64},  {192, 192, 64},
        {64, 64, 192},   {192, 64, 192}, {64, 192, 192}, {192, 192, 192},
        {32, 0, 0},      {160, 0, 0},    {32, 128, 0},   {160, 128, 0},
        {32, 0, 128},    {160, 0, 128},  {32, 128, 128}, {160, 128, 128},
        {96, 0, 0},      {224, 0, 0},    {96, 128, 0},   {224, 128, 0},
        {96, 0, 128},    {224, 0, 128},  {96, 128, 128}, {224, 128, 128},
        {32, 64, 0},     {160, 64, 0},   {32, 192, 0},   {160, 192, 0},
        {32, 64, 128},   {160, 64, 128}, {32, 192, 128}, {160, 192, 128},
        {96, 64, 0},     {224, 64, 0},   {96, 192, 0},   {224, 192, 0},
        {96, 64, 128},   {224, 64, 128}, {96, 192, 128}, {224, 192, 128},
        {32, 0, 64},     {160, 0, 64},   {32, 128, 64},  {160, 128, 64},
        {32, 0, 192},    {160, 0, 192},  {32, 128, 192}, {160, 128, 192},
        {96, 0, 64},     {224, 0, 64},   {96, 128, 64},  {224, 128, 64},
        {96, 0, 192},    {224, 0, 192},  {96, 128, 192}, {224, 128, 192},
        {32, 64, 64},    {160, 64, 64},  {32, 192, 64},  {160, 192, 64},
        {32, 64, 192},   {160, 64, 192}, {32, 192, 192}, {160, 192, 192},
        {96, 64, 64},    {224, 64, 64},  {96, 192, 64},  {224, 192, 64},
        {96, 64, 192},   {224, 64, 192}, {96, 192, 192}, {224, 192, 192},
        {0, 32, 0},      {128, 32, 0},   {0, 160, 0},    {128, 160, 0},
        {0, 32, 128},    {128, 32, 128}, {0, 160, 128},  {128, 160, 128},
        {64, 32, 0},     {192, 32, 0},   {64, 160, 0},   {192, 160, 0},
        {64, 32, 128},   {192, 32, 128}, {64, 160, 128}, {192, 160, 128},
        {0, 96, 0},      {128, 96, 0},   {0, 224, 0},    {128, 224, 0},
        {0, 96, 128},    {128, 96, 128}, {0, 224, 128},  {128, 224, 128},
        {64, 96, 0},     {192, 96, 0},   {64, 224, 0},   {192, 224, 0},
        {64, 96, 128},   {192, 96, 128}, {64, 224, 128}, {192, 224, 128},
        {0, 32, 64},     {128, 32, 64},  {0, 160, 64},   {128, 160, 64},
        {0, 32, 192},    {128, 32, 192}, {0, 160, 192},  {128, 160, 192},
        {64, 32, 64},    {192, 32, 64},  {64, 160, 64},  {192, 160, 64},
        {64, 32, 192},   {192, 32, 192}, {64, 160, 192}, {192, 160, 192},
        {0, 96, 64},     {128, 96, 64},  {0, 224, 64},   {128, 224, 64},
        {0, 96, 192},    {128, 96, 192}, {0, 224, 192},  {128, 224, 192},
        {64, 96, 64},    {192, 96, 64},  {64, 224, 64},  {192, 224, 64},
        {64, 96, 192},   {192, 96, 192}, {64, 224, 192}, {192, 224, 192},
        {32, 32, 0},     {160, 32, 0},   {32, 160, 0},   {160, 160, 0},
        {32, 32, 128},   {160, 32, 128}, {32, 160, 128}, {160, 160, 128},
        {96, 32, 0},     {224, 32, 0},   {96, 160, 0},   {224, 160, 0},
        {96, 32, 128},   {224, 32, 128}, {96, 160, 128}, {224, 160, 128},
        {32, 96, 0},     {160, 96, 0},   {32, 224, 0},   {160, 224, 0},
        {32, 96, 128},   {160, 96, 128}, {32, 224, 128}, {160, 224, 128},
        {96, 96, 0},     {224, 96, 0},   {96, 224, 0},   {224, 224, 0},
        {96, 96, 128},   {224, 96, 128}, {96, 224, 128}, {224, 224, 128},
        {32, 32, 64},    {160, 32, 64},  {32, 160, 64},  {160, 160, 64},
        {32, 32, 192},   {160, 32, 192}, {32, 160, 192}, {160, 160, 192},
        {96, 32, 64},    {224, 32, 64},  {96, 160, 64},  {224, 160, 64},
        {96, 32, 192},   {224, 32, 192}, {96, 160, 192}, {224, 160, 192},
        {32, 96, 64},    {160, 96, 64},  {32, 224, 64},  {160, 224, 64},
        {32, 96, 192},   {160, 96, 192}, {32, 224, 192}, {160, 224, 192},
        {96, 96, 64},    {224, 96, 64},  {96, 224, 64},  {224, 224, 64},
        {96, 96, 192},   {224, 96, 192}, {96, 224, 192}, {224, 224, 192}};

    coco_color_code[61] = {192, 128, 192};
    coco_color_code[57] = {192, 64, 64};
    coco_color_code[14] = {64, 128, 128};
    coco_color_code[63] = {0, 64, 192};
    coco_color_code[25] = {128, 64, 192};
    coco_color_code[42] = {64, 128, 64};
    coco_color_code[29] = {128, 192, 192};
    coco_color_code[65] = {64, 0, 128};
    coco_color_code[67] = {160, 128, 0};
    coco_color_code[69] = {160, 0, 128};
    coco_color_code[59] = {64, 64, 128};
    coco_color_code[73] = {64, 192, 0};

    Color color;
    color.r = coco_color_code.at(semantic_label)[0];
    color.g = coco_color_code.at(semantic_label)[1];
    color.b = coco_color_code.at(semantic_label)[2];

    // color.r = nyu_color_code.at(semantic_label)[0];
    // color.g = nyu_color_code.at(semantic_label)[1];
    // color.b = nyu_color_code.at(semantic_label)[2];

    // if (semantic_label == 0) [
    //   color.a = 0.5;
    // }

    // uint8_t ind = semantic_label;
    // color.r = 0;
    // color.g = 0;
    // color.b = 0;
    // for (int i = 7; i >= 0; --i) {
    //   color.r |= (((ind >> 0) & 1) << i);
    //   color.g |= (((ind >> 1) & 1) << i);
    //   color.b |= (((ind >> 2) & 1) << i);
    //   ind >>= 3;
    // }

    // color.r = 0;
    // color.g = 0;
    // color.b = 0;
    // uint8_t c = semantic_label;
    // for (int i = 7; i >= 0; --i) {
    //   color.r |= ((c & (1 << 0)) << i);
    //   color.g |= ((c & (1 << 1)) << i);
    //   color.b |= ((c & (1 << 2)) << i);
    //   c >>= 3;
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
  const std::map<Label, int>* label_age_map_ptr_;

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
