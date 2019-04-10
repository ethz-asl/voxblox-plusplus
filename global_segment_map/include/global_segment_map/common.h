#ifndef GLOBAL_SEGMENT_MAP_COMMON_H_
#define GLOBAL_SEGMENT_MAP_COMMON_H_

#include <map>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "voxblox/core/common.h"

namespace voxblox {

// GSM custom types.
typedef uint16_t Label;
typedef uint16_t LabelConfidence;
typedef uint8_t SemanticLabel;
typedef uint8_t InstanceLabel;

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

struct PointSurfelSemanticInstance : public pcl::PointXYZRGB {
  PCL_ADD_POINT4D;
  PCL_ADD_RGB;
  // TODO(margaritaG): Fix field names to:
  SemanticLabel label;     // semantic_label
  InstanceLabel instance;  // instance_label

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

typedef pcl::PointXYZRGB PointType;
typedef PointSurfelSemanticInstance PointSemanticInstanceType;

// Pointcloud types for external interface.
typedef AlignedVector<Label> Labels;
typedef AlignedVector<SemanticLabel> SemanticLabels;

}  // namespace voxblox

POINT_CLOUD_REGISTER_POINT_STRUCT(
    voxblox::PointSemanticInstanceType,
    (float, x, x)(float, y, y)(float, z, z)(float, rgb, rgb)(
        voxblox::SemanticLabel, label,
        label)(voxblox::InstanceLabel, instance,
               instance))  // TODO(margaritaG): Fix field names

#endif  // GLOBAL_SEGMENT_MAP_COMMON_H_
