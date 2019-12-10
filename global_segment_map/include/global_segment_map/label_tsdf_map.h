#ifndef GLOBAL_SEGMENT_MAP_LABEL_TSDF_MAP_H_
#define GLOBAL_SEGMENT_MAP_LABEL_TSDF_MAP_H_

#include <memory>
#include <utility>

#include <glog/logging.h>
#include <voxblox/core/common.h>
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>

#include "global_segment_map/label_voxel.h"
#include "global_segment_map/semantic_instance_label_fusion.h"

namespace voxblox {

class LabelTsdfMap {
 public:
  typedef std::shared_ptr<LabelTsdfMap> Ptr;

  typedef std::pair<Layer<TsdfVoxel>, Layer<LabelVoxel>> LayerPair;

  struct Config {
    FloatingPoint voxel_size = 0.2;
    size_t voxels_per_side = 16u;
  };

  explicit LabelTsdfMap(const Config& config)
      : tsdf_layer_(
            new Layer<TsdfVoxel>(config.voxel_size, config.voxels_per_side)),
        label_layer_(
            new Layer<LabelVoxel>(config.voxel_size, config.voxels_per_side)),
        config_(config),
        highest_label_(0u),
        highest_instance_(0u) {}

  virtual ~LabelTsdfMap() {}

  inline Layer<TsdfVoxel>* getTsdfLayerPtr() { return tsdf_layer_.get(); }
  inline const Layer<TsdfVoxel>& getTsdfLayer() const { return *tsdf_layer_; }

  inline Layer<LabelVoxel>* getLabelLayerPtr() { return label_layer_.get(); }
  inline const Layer<LabelVoxel>& getLabelLayer() const {
    return *label_layer_;
  }

  inline LMap* getLabelCountPtr() { return &label_count_map_; }

  inline Label* getHighestLabelPtr() { return &highest_label_; }

  inline InstanceLabel* getHighestInstancePtr() { return &highest_instance_; }

  inline SemanticInstanceLabelFusion* getSemanticInstanceLabelFusionPtr() {
    return &semantic_instance_label_fusion_;
  }
  inline const SemanticInstanceLabelFusion& getSemanticInstanceLabelFusion()
      const {
    return semantic_instance_label_fusion_;
  }

  inline FloatingPoint block_size() const { return tsdf_layer_->block_size(); }

  // Get the list of all labels
  // for which the voxel count is greater than 0.
  // NOT THREAD SAFE.
  Labels getLabelList();

  // Get the list of all instance labels
  // for which the voxel count is greater than 0.
  // NOT THREAD SAFE.
  InstanceLabels getInstanceList();

  // Get the list of semantic categories of all instances
  // for which the voxel count is greated than 0.
  // NOT THREAD SAFE.
  void getSemanticInstanceList(InstanceLabels* instance_labels,
                               SemanticLabels* semantic_labels);

  /**
   * Extracts separate tsdf and label layers from the gsm, for every given
   * label.
   * @param labels of segments to extract
   * @param label_layers_map output map
   * @param labels_list_is_complete true if the gsm does not contain other
   * labels. false if \labels is only a subset of all labels contained by
   * the gsm.
   */
  void extractSegmentLayers(
      const Labels& labels,
      std::unordered_map<Label, LayerPair>* label_layers_map,
      const bool labels_list_is_complete = false);

  void extractInstanceLayers(
      const InstanceLabels& instance_labels,
      std::unordered_map<InstanceLabel, LayerPair>* instance_layers_map);

 protected:
  Config config_;

  // The layers.
  Layer<TsdfVoxel>::Ptr tsdf_layer_;
  Layer<LabelVoxel>::Ptr label_layer_;

  // Bookkeping.
  Label highest_label_;
  LMap label_count_map_;
  InstanceLabel highest_instance_;

  // Semantic instance-aware segmentation.
  SemanticInstanceLabelFusion semantic_instance_label_fusion_;
};

}  // namespace voxblox

#endif  // GLOBAL_SEGMENT_MAP_LABEL_TSDF_MAP_H_
