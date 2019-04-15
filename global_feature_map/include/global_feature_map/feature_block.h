#ifndef GLOBAL_FEATURE_MAP_FEATURE_BLOCK_H_
#define GLOBAL_FEATURE_MAP_FEATURE_BLOCK_H_

#include <algorithm>
#include <atomic>
#include <memory>
#include <vector>

#include <voxblox/core/common.h>

namespace voxblox {

/** A 3D block container that contains 3D features for the encompassing 3D
 * space. */
template <typename FeatureType>
class FeatureBlock {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<FeatureBlock<FeatureType> > Ptr;
  typedef std::shared_ptr<const FeatureBlock<FeatureType> > ConstPtr;

  FeatureBlock(FloatingPoint block_size, const Point& origin)
      : has_data_(false),
        block_size_(block_size),
        origin_(origin),
        updated_(false) {
    CHECK_GT(block_size_, 0.f);
    block_size_inv_ = 1.0 / block_size_;
  }

  ~FeatureBlock() {}

  BlockIndex block_index() const {
    return getGridIndexFromOriginPoint<BlockIndex>(origin_, block_size_inv_);
  }

  size_t num_features() const { return features_.size(); }
  Point origin() const { return origin_; }
  void setOrigin(const Point& new_origin) { origin_ = new_origin; }
  FloatingPoint block_size() const { return block_size_; }

  inline const std::vector<FeatureType>& features() const { return features_; }

  bool has_data() const { return has_data_; }
  bool updated() const { return updated_; }

  std::atomic<bool>& updated() { return updated_; }
  bool& has_data() { return has_data_; }

  void set_updated(bool updated) { updated_ = updated; }
  void set_has_data(bool has_data) { has_data_ = has_data; }

  const FeatureType& getFeature(const size_t index) const {
    CHECK_LT(index, features_.size());
    return features_[index];
  }

  FeatureType& getFeature(const size_t index) {
    CHECK_LT(index, features_.size());
    return features_[index];
  }

  std::vector<FeatureType>& getFeatures() { return features_; }

  void addFeature(const FeatureType& feature) { features_.push_back(feature); }

  void mergeBlock(const FeatureBlock<FeatureType>& other_block);

  size_t getMemorySize() const;

 protected:
  std::vector<FeatureType> features_;

  bool has_data_;

 private:
  // Base parameters.
  Point origin_;

  // Derived, cached parameters.
  FloatingPoint block_size_;
  FloatingPoint block_size_inv_;

  /// Is set to true when data is updated.
  std::atomic<bool> updated_;
};

}  // namespace voxblox

#include "global_feature_map/feature_block_inl.h"

#endif  // GLOBAL_FEATURE_MAP_FEATURE_BLOCK_H_
