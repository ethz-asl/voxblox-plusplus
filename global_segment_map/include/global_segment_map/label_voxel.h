#ifndef GLOBAL_SEGMENT_MAP_LABEL_VOXEL_H_
#define GLOBAL_SEGMENT_MAP_LABEL_VOXEL_H_

#include <cstdint>

#include <pcl/point_types.h>
#include <voxblox/core/color.h>
#include <voxblox/core/common.h>
#include <voxblox/core/voxel.h>

struct PointSurfelLabel {
  PCL_ADD_POINT4D;
  PCL_ADD_NORMAL4D;
  PCL_ADD_RGB;
  uint32_t label;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointSurfelLabel,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                      float, rgb, rgb)(uint32_t, label, label))

namespace voxblox {

typedef PointSurfelLabel PointType;

typedef uint16_t Label;
typedef uint16_t LabelConfidence;
typedef uint8_t SemanticLabel;

// Pointcloud types for external interface.
typedef AlignedVector<Label> Labels;
typedef AlignedVector<SemanticLabel> SemanticLabels;

struct LabelCount {
  Label label = 0u;
  LabelConfidence label_confidence = 0.0f;
};

struct LabelVoxel {
  Label label = 0u;
  LabelConfidence label_confidence = 0.0f;
  LabelCount label_count[40];
};

namespace voxel_types {
const std::string kLabel = "label";
}  // namespace voxel_types

template <>
inline std::string getVoxelType<LabelVoxel>() {
  return voxel_types::kLabel;
}

}  // namespace voxblox

#endif  // GLOBAL_SEGMENT_MAP_LABEL_VOXEL_H_
