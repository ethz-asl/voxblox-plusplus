#ifndef GLOBAL_SEGMENT_MAP_LABEL_TSDF_MAP_H_
#define GLOBAL_SEGMENT_MAP_LABEL_TSDF_MAP_H_

#include <memory>
#include <utility>

#include <glog/logging.h>
#include <voxblox/core/common.h>
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>

#include "global_segment_map/label_voxel.h"

namespace voxblox {

class LabelTsdfMap {
 public:
  typedef std::shared_ptr<LabelTsdfMap> Ptr;

  struct Config {
    FloatingPoint voxel_size = 0.2;
    size_t voxels_per_side = 16u;
  };

  explicit LabelTsdfMap(const Config& config)
      : tsdf_layer_(
            new Layer<TsdfVoxel>(config.voxel_size, config.voxels_per_side)),
        label_layer_(
            new Layer<LabelVoxel>(config.voxel_size, config.voxels_per_side)),
        highest_label_(0u),
        highest_instance_(0u) {}

  virtual ~LabelTsdfMap() {}

  Layer<TsdfVoxel>* getTsdfLayerPtr() { return tsdf_layer_.get(); }

  const Layer<TsdfVoxel>& getTsdfLayer() const { return *tsdf_layer_; }

  Layer<LabelVoxel>* getLabelLayerPtr() { return label_layer_.get(); }

  const Layer<LabelVoxel>& getLabelLayer() const { return *label_layer_; }

  Label* getHighestLabelPtr() { return &highest_label_; }

  InstanceLabel* getHighestInstancePtr() { return &highest_instance_; }

  FloatingPoint block_size() const { return tsdf_layer_->block_size(); }

 protected:
  Label highest_label_;
  InstanceLabel highest_instance_;

  // The layers.
  Layer<TsdfVoxel>::Ptr tsdf_layer_;
  Layer<LabelVoxel>::Ptr label_layer_;
};

}  // namespace voxblox

#endif  // GLOBAL_SEGMENT_MAP_LABEL_TSDF_MAP_H_
