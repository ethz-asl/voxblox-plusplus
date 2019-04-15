#ifndef GLOBAL_FEATURE_MAP_FEATURE_TYPES_H_
#define GLOBAL_FEATURE_MAP_FEATURE_TYPES_H_

#include <string>

#include <pcl/point_types.h>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <voxblox/core/common.h>

namespace voxblox {

struct Feature3D {
  pcl::PointSurfel keypoint;
  double keypoint_scale;
  double keypoint_response;
  cv::Mat descriptor;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <typename Type>
inline std::string getFeatureType() {
  return "Unknown";
};

template <>
std::string getFeatureType<Feature3D>();

}  // namespace voxblox

#endif  // GLOBAL_FEATURE_MAP_FEATURE_TYPES_H_
