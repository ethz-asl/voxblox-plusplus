#ifndef GLOBAL_SEGMENT_MAP_UTILS_LABEL_UTILS_H_
#define GLOBAL_SEGMENT_MAP_UTILS_LABEL_UTILS_H_

#include "global_segment_map/label_voxel.h"

namespace voxblox {
namespace utils {

inline SemanticLabel getSemanticLabel(
    const std::map<Label, std::map<SemanticLabel, int>>& label_class_count,
    const Label& label) {
  SemanticLabel semantic_label = 0;
  int max_count = 0;
  auto label_it = label_class_count.find(label);
  if (label_it != label_class_count.end()) {
    for (auto const& class_count : label_it->second) {
      if (class_count.second > max_count && class_count.first != 0u) {
        semantic_label = class_count.first;
        max_count = class_count.second;
      }
    }
  }
  return semantic_label;
}

}  // namespace utils
}  // namespace voxblox

#endif  // GLOBAL_SEGMENT_MAP_UTILS_LABEL_UTILS_H_
