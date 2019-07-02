#ifndef GLOBAL_SEGMENT_MAP_MESHING_LABEL_COLOR_MAP_H_
#define GLOBAL_SEGMENT_MAP_MESHING_LABEL_COLOR_MAP_H_

#include <shared_mutex>

#include "global_segment_map/common.h"

namespace voxblox {

class LabelColorMap {
 public:
  void getColor(const Label& label, Color* color);

 protected:
  std::map<Label, Color> color_map_;
  std::shared_timed_mutex color_map_mutex_;
};
}  // namespace voxblox

#endif
