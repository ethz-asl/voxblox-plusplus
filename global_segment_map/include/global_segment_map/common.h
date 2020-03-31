#ifndef GLOBAL_SEGMENT_MAP_COMMON_H_
#define GLOBAL_SEGMENT_MAP_COMMON_H_

#include <map>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "voxblox/core/common.h"

namespace voxblox {

const uint16_t BackgroundLabel = 0u;

// Voxblox++ custom types.
typedef uint16_t Label;
typedef uint16_t LabelConfidence;
typedef uint16_t InstanceLabel;
typedef uint8_t SemanticLabel;

typedef std::vector<Label> Labels;
typedef std::vector<SemanticLabel> SemanticLabels;
typedef std::vector<InstanceLabel> InstanceLabels;

typedef std::map<Label, int> LMap;
typedef std::map<Label, int>::iterator LMapIt;
typedef std::map<Label, LMap> LLMap;
typedef std::map<Label, LMap>::iterator LLMapIt;
typedef std::set<Label> LSet;
typedef std::set<Label>::iterator LSetIt;
typedef std::map<Label, LSet> LLSet;
typedef std::map<Label, LSet>::iterator LLSetIt;
typedef std::map<SemanticLabel, int> SLMap;
typedef std::map<Label, SLMap> LSLMap;

struct LabelCount {
  Label label = 0u;
  LabelConfidence label_confidence = 0u;
};

struct PointSurfelLabel {
  PCL_ADD_POINT4D;
  PCL_ADD_NORMAL4D;
  PCL_ADD_RGB;
  uint32_t label;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

struct PointSurfelSemanticInstance {
  PCL_ADD_POINT4D;
  PCL_ADD_NORMAL4D;
  PCL_ADD_RGB;
  uint8_t instance_label;
  SemanticLabel semantic_label;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// Map pointcloud type.
struct PointTSDFLabel {
  PCL_ADD_POINT4D;

  // TSDF fields.
  float distance;
  float weight;

  // Semantic segmentation fields.
  Label segment_label;
  SemanticLabel semantic_class;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

typedef pcl::PointXYZRGB PointType;
typedef PointSurfelLabel PointLabelType;
typedef PointSurfelSemanticInstance PointSemanticInstanceType;
typedef PointTSDFLabel PointMapType;

}  // namespace voxblox

POINT_CLOUD_REGISTER_POINT_STRUCT(
    voxblox::PointSurfelLabel,
    (float, x, x)(float, y, y)(float, z, z)(float, rgb, rgb)(std::uint32_t,
                                                             label, label))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    voxblox::PointSemanticInstanceType,
    (float, x, x)(float, y, y)(float, z, z)(float, rgb, rgb)(
        std::uint8_t, instance_label,
        instance_label)(voxblox::SemanticLabel, semantic_label, semantic_label))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    voxblox::PointTSDFLabel,
    (float, x, x)(float, y, y)(float, z, z)(float, distance, distance)(
        float, weight, weight)(voxblox::Label, segment_label,
                               segment_label)(voxblox::SemanticLabel,
                                              semantic_class, semantic_class))

#endif  // GLOBAL_SEGMENT_MAP_COMMON_H_
