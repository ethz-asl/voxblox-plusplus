#ifndef GLOBAL_SEGMENT_MAP_LABEL_FUSION_H_
#define GLOBAL_SEGMENT_MAP_LABEL_FUSION_H_

#include <map>

#include "global_segment_map/common.h"

namespace voxblox {
namespace utils {

class InstanceLabelFusion {
 public:
  void increaseLabelInstanceCount(const Label& label,
                                  const InstanceLabel& instance_label);

  void decreaseLabelInstanceCount(const Label& label,
                                  const InstanceLabel& instance_label);

  void increaseLabelFramesCount(const Label& label);

  InstanceLabel getLabelInstance(
      const Label& label, const std::set<InstanceLabel>& assigned_instances =
                              std::set<InstanceLabel>()) const;

 protected:
  LSLMap label_instance_count_;
  LMap label_frames_count_;
};

class SemanticLabelFusion {
 public:
  void increaseLabelClassCount(const Label& label,
                               const SemanticLabel& semantic_label);

  SemanticLabel getSemanticLabel(const Label& label) const;

 protected:
  // Per frame voxel count of semantic label.
  LSLMap label_class_count_;
};

}  // namespace utils
}  // namespace voxblox

#endif  // GLOBAL_SEGMENT_MAP_LABEL_FUSION_H_
