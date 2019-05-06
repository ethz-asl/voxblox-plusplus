#ifndef GLOBAL_FEATURE_MAP_FEATURE_BLOCK_H_
#define GLOBAL_FEATURE_MAP_FEATURE_BLOCK_H_

#include <algorithm>
#include <atomic>
#include <memory>
#include <mutex>
#include <vector>

#include <voxblox/core/common.h>
#include "./FeatureBlock.pb.h"

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

  explicit FeatureBlock(const FeatureBlockProto& proto,
                        const size_t descriptor_size);

  ~FeatureBlock() {}

  inline BlockIndex block_index() const {
    return getGridIndexFromOriginPoint<BlockIndex>(origin_, block_size_inv_);
  }

  inline FloatingPoint block_size() const { return block_size_; }

  inline Point origin() const { return origin_; }
  inline void setOrigin(const Point& new_origin) { origin_ = new_origin; }

  inline void set_updated(bool updated) { updated_ = updated; }
  inline std::atomic<bool>& updated() { return updated_; }
  inline bool updated() const { return updated_; }

  inline void set_has_data(bool has_data) { has_data_ = has_data; }
  inline bool& has_data() { return has_data_; }
  inline bool has_data() const { return has_data_; }

  inline std::vector<FeatureType>& getFeatures() { return features_; }
  inline const std::vector<FeatureType>& getFeatures() const {
    return features_;
  }
  inline void setFeatures(const std::vector<FeatureType>& features) {
    features_ = features;
  }

  inline const FeatureType& getFeature(const size_t index) const {
    CHECK_LT(index, features_.size());
    return features_[index];
  }
  inline FeatureType& getFeature(const size_t index) {
    CHECK_LT(index, features_.size());
    return features_[index];
  }
  inline void addFeature(const FeatureType& feature) {
    features_.push_back(feature);
    set_has_data(true);
  }

  inline size_t numFeatures() const { return features_.size(); }

  inline void shiftFeatures(const Point& shift) {
    for (FeatureType& feature : features_) {
      feature.keypoint -= shift;
    }
  }

  void mergeBlock(const FeatureBlock<FeatureType>& other_block);

  size_t getMemorySize() const;

  inline std::mutex& getMutex() { return block_mutex_; }

  void serializeToIntegers(const size_t descriptor_size,
                           std::vector<uint32_t>* data) const;

  void deserializeFromIntegers(const size_t descriptor_size,
                               const std::vector<uint32_t>& data);

  void getProto(const size_t descriptor_size, FeatureBlockProto* proto) const;

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

  std::mutex block_mutex_;
};

}  // namespace voxblox

#endif  // GLOBAL_FEATURE_MAP_FEATURE_BLOCK_H_

#include "global_feature_map/feature_block_inl.h"
