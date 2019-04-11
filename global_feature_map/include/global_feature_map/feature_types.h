#ifndef GLOBAL_FEATURE_MAP_FEATURE_TYPES_H_
#define GLOBAL_FEATURE_MAP_FEATURE_TYPES_H_

#include <string>

#include <voxblox/core/common.h>

namespace voxblox {

struct Feature3D {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Point point;

  // Add feature stuff.
};

template <typename Type>
std::string getFeatureType();

template <>
std::string getFeatureType<Feature3D>();

}  // namespace voxblox

#endif  // GLOBAL_FEATURE_MAP_FEATURE_TYPES_H_
