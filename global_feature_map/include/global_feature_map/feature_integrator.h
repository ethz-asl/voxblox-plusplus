#ifndef VOXBLOX_GSM_GLOBAL_FEATURE_MAP_FEATURE_INTEGRATOR_H_
#define VOXBLOX_GSM_GLOBAL_FEATURE_MAP_FEATURE_INTEGRATOR_H_

#include <mutex>
#include <thread>
#include <vector>

#include <glog/logging.h>
#include <voxblox/core/common.h>
#include <voxblox/integrator/integrator_utils.h>
#include <voxblox/utils/approx_hash_array.h>
#include <voxblox/utils/timing.h>
#include <Eigen/Core>

#include "global_feature_map/feature_block.h"
#include "global_feature_map/feature_layer.h"
#include "global_feature_map/feature_types.h"

namespace voxblox {

// TODO(ntonci): Template with FeatureType
class FeatureIntegrator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<FeatureIntegrator> Ptr;

  struct Config {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    size_t integrator_threads = std::thread::hardware_concurrency();

    // Number of features to keep, if set to 0, all features are stored and
    // sorting is not performed.
    size_t number_of_features_to_keep = 0u;
  };

  FeatureIntegrator(const Config& config, FeatureLayer<Feature3D>* layer)
      : config_(config) {
    setLayer(layer);

    if (config_.integrator_threads == 0u) {
      LOG(WARNING) << "Automatic core count failed, defaulting to 1 threads";
      config_.integrator_threads = 1u;
    }
  }

  inline void setLayer(FeatureLayer<Feature3D>* layer) {
    CHECK_NOTNULL(layer);

    layer_ = layer;
    block_size_ = layer_->block_size();
    block_size_inv_ = 1.0 / block_size_;
  }

  inline const Config& getConfig() const { return config_; }

  void integrateFeatures(const Transformation& T_G_C,
                         const std::vector<Feature3D>& features);

 protected:
  void integrateFunction(const Transformation& T_G_C,
                         const std::vector<Feature3D>& features,
                         ThreadSafeIndex* index_getter);

  void allocateStorageAndGetBlockPtr(const Point& point_G,
                                     FeatureBlock<Feature3D>::Ptr* last_block,
                                     BlockIndex* last_block_idx);

  void updateFeatureBlock(const Feature3D& feature,
                          FeatureBlock<Feature3D>::Ptr* block);

  void updateLayerWithStoredBlocks();

  Config config_;
  FeatureLayer<Feature3D>* layer_;

  FloatingPoint block_size_;
  FloatingPoint block_size_inv_;

  std::mutex temp_block_mutex_;
  FeatureLayer<Feature3D>::FeatureBlockHashMap temp_block_map_;
};

}  // namespace voxblox

#endif  // VOXBLOX_GSM_GLOBAL_FEATURE_MAP_FEATURE_INTEGRATOR_H_
