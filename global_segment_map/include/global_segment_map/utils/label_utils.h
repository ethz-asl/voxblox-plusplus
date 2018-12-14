#ifndef GLOBAL_SEGMENT_MAP_UTILS_LABEL_UTILS_H_
#define GLOBAL_SEGMENT_MAP_UTILS_LABEL_UTILS_H_

#include <map>

#include "global_segment_map/label_voxel.h"

namespace voxblox {
namespace utils {

SemanticLabel getSemanticLabel(
    const std::map<Label, std::map<SemanticLabel, int>>& label_class_count,
    const Label& label);

}  // namespace utils
}  // namespace voxblox

#endif  // GLOBAL_SEGMENT_MAP_UTILS_LABEL_UTILS_H_
