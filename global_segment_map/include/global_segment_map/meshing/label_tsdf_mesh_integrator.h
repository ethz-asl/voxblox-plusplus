#ifndef GLOBAL_SEGMENT_MAP_LABEL_TSDF_MESH_INTEGRATOR_H_
#define GLOBAL_SEGMENT_MAP_LABEL_TSDF_MESH_INTEGRATOR_H_

#include <algorithm>
#include <cmath>
#include <list>
#include <map>
#include <vector>

#include <glog/logging.h>
#include "voxblox/core/color.h"
#include "voxblox/mesh/mesh_integrator.h"

#include "global_segment_map/label_tsdf_map.h"
#include "global_segment_map/label_voxel.h"
#include "global_segment_map/meshing/instance_color_map.h"
#include "global_segment_map/meshing/label_color_map.h"
#include "global_segment_map/meshing/semantic_color_map.h"
#include "global_segment_map/semantic_instance_label_fusion.h"
#include "global_segment_map/utils/meshing_utils.h"

namespace voxblox {

class MeshLabelIntegrator : public MeshIntegrator<TsdfVoxel> {
 public:
  enum ColorScheme {
    kColor = 0,
    kNormals,
    kLabel,
    kLabelConfidence,
    kSemantic,
    kInstance,
    kMerged
  };

  struct LabelTsdfConfig {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LabelConfidence max_confidence = 200u;

    // Color scheme to use for the mesh color.
    // By default the mesh encodes the labels.
    ColorScheme color_scheme = ColorScheme::kLabel;
    SemanticColorMap::ClassTask class_task =
        SemanticColorMap::ClassTask::kCoco80;
  };

  MeshLabelIntegrator(
      const MeshIntegratorConfig& config,
      const MeshLabelIntegrator::LabelTsdfConfig& label_tsdf_config,
      LabelTsdfMap* map, MeshLayer* mesh_layer,
      std::set<SemanticLabel>& all_semantic_labels, bool* remesh = nullptr);

  MeshLabelIntegrator(
      const MeshIntegratorConfig& config,
      const MeshLabelIntegrator::LabelTsdfConfig& label_tsdf_config,
      const LabelTsdfMap& map, MeshLayer* mesh_layer,
      std::set<SemanticLabel>& all_semantic_labels, bool* remesh = nullptr);

  MeshLabelIntegrator(
      const MeshIntegratorConfig& config,
      const MeshLabelIntegrator::LabelTsdfConfig& label_tsdf_config,
      const Layer<TsdfVoxel>& tsdf_layer, const Layer<LabelVoxel>& label_layer,
      MeshLayer* mesh_layer);

  // Generates mesh for the tsdf layer.
  bool generateMesh(bool only_mesh_updated_blocks, bool clear_updated_flag);

 protected:
  void generateMeshBlocksFunction(const BlockIndexList& all_tsdf_blocks,
                                  bool clear_updated_flag,
                                  ThreadSafeIndex* index_getter);

  InstanceLabel getInstanceLabel(const Label& label);

  virtual void updateMeshForBlock(const BlockIndex& block_index);

  void updateMeshBlockColor(Block<TsdfVoxel>::ConstPtr tsdf_block,
                            Block<LabelVoxel>::ConstPtr label_block,
                            Mesh* mesh_block);

  void updateMeshColor(const Block<TsdfVoxel>& tsdf_block, Mesh* mesh);

  void updateMeshColor(const Block<LabelVoxel>& label_block, Mesh* mesh);

  LabelTsdfConfig label_tsdf_config_;

  // Having both a const and a mutable pointer to the layer allows this
  // integrator to work both with a const layer (in case you don't want to
  // clear the updated flag) and mutable layer (in case you do want to clear
  // the updated flag).
  Layer<LabelVoxel>* label_layer_mutable_ptr_;
  const Layer<LabelVoxel>* label_layer_const_ptr_;

  const SemanticInstanceLabelFusion* semantic_instance_label_fusion_ptr_;

  // const std::map<Label, std::map<SemanticLabel, int>>*
  // label_class_count_ptr_; const std::map<Label, std::map<SemanticLabel,
  // int>>* label_instance_count_ptr_;
  // const std::map<Label, int>* label_frames_count_ptr_;
  std::set<SemanticLabel>* all_semantic_labels_ptr_;
  bool* remesh_ptr_;
  // This parameter is used if no valid remesh_ptr is provided to the class at
  // construction time.
  bool remesh_ = false;
  std::map<Label, InstanceLabel> label_instance_map_;

  LabelColorMap label_color_map_;
  SemanticColorMap semantic_color_map_;
  InstanceColorMap instance_color_map_;
};

}  // namespace voxblox

#endif  // GLOBAL_SEGMENT_MAP_LABEL_TSDF_MESH_INTEGRATOR_H_
