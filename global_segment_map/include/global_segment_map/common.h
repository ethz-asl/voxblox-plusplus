#ifndef GLOBAL_SEGMENT_MAP_COMMON_H_
#define GLOBAL_SEGMENT_MAP_COMMON_H_

#include <pcl/point_types.h>

#include "voxblox/core/common.h"

namespace voxblox {

// GSM custom types.
typedef uint16_t Label;
typedef uint16_t LabelConfidence;
typedef uint8_t SemanticLabel;
typedef uint8_t InstanceLabel;

struct LabelCount {
  Label label = 0u;
  LabelConfidence label_confidence = 0u;
};

struct PointSurfelSemanticInstance {
  PCL_ADD_POINT4D;
  PCL_ADD_NORMAL4D;
  PCL_ADD_RGB;
  // TODO(margaritaG): Fix field names
  SemanticLabel label;
  InstanceLabel instance;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

typedef PointSurfelSemanticInstance PointType;

// Pointcloud types for external interface.
typedef AlignedVector<Label> Labels;
typedef AlignedVector<SemanticLabel> SemanticLabels;

}  // namespace voxblox

POINT_CLOUD_REGISTER_POINT_STRUCT(
    voxblox::PointSurfelSemanticInstance,
    (float, x, x)(float, y, y)(float, z, z)(float, normal_x, normal_x)(
        float, normal_y, normal_y)(float, normal_z, normal_z)(float, rgb, rgb)(
        voxblox::SemanticLabel, label,
        label)(voxblox::InstanceLabel, instance,
               instance))  // TODO(margaritaG): Fix field names

#endif  // GLOBAL_SEGMENT_MAP_COMMON_H_
