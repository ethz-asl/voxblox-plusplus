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

#ifndef GLOBAL_SEGMENT_MAP_LABEL_TSDF_MESH_INTEGRATOR_H_
#define GLOBAL_SEGMENT_MAP_LABEL_TSDF_MESH_INTEGRATOR_H_

#include <algorithm>
#include <cmath>
#include <list>
#include <map>

#include <glog/logging.h>
#include <voxblox/core/color.h>
#include <voxblox/mesh/mesh_integrator.h>

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
      LabelTsdfMap* map, MeshLayer* mesh_layer, bool* remesh = nullptr);

  MeshLabelIntegrator(
      const MeshIntegratorConfig& config,
      const MeshLabelIntegrator::LabelTsdfConfig& label_tsdf_config,
      const LabelTsdfMap& map, MeshLayer* mesh_layer, bool* remesh = nullptr);

  MeshLabelIntegrator(
      const MeshIntegratorConfig& config,
      const MeshLabelIntegrator::LabelTsdfConfig& label_tsdf_config,
      const Layer<TsdfVoxel>& tsdf_layer, const Layer<LabelVoxel>& label_layer,
      MeshLayer* mesh_layer);

  // Generates mesh for the tsdf layer.
  bool generateMesh(const bool only_mesh_updated_blocks,
                    const bool clear_updated_flag);

 protected:
  void generateMeshBlocksFunction(const BlockIndexList& all_tsdf_blocks,
                                  const bool clear_updated_flag,
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

  bool* remesh_ptr_;
  // This parameter is used if no valid remesh_ptr is provided to the class at
  // construction time.
  bool remesh_ = false;
  std::map<Label, InstanceLabel> label_instance_map_;
  std::shared_timed_mutex label_instance_map_mutex_;

  LabelColorMap label_color_map_;
  SemanticColorMap semantic_color_map_;
  InstanceColorMap instance_color_map_;
};

}  // namespace voxblox

#endif  // GLOBAL_SEGMENT_MAP_LABEL_TSDF_MESH_INTEGRATOR_H_
