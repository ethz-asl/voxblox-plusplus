#include "global_segment_map/utils/label_utils.h"

#include <map>

#include "global_segment_map/label_voxel.h"

namespace voxblox {
namespace utils {

SemanticLabel getSemanticLabel(
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
  } else {
    // LOG(ERROR) << "No semantic class for label?";
  }
  return semantic_label;
}

}  // namespace utils
}  // namespace voxblox
