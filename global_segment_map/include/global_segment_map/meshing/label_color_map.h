#ifndef GLOBAL_SEGMENT_MAP_MESHING_LABEL_COLOR_MAP_H_
#define GLOBAL_SEGMENT_MAP_MESHING_LABEL_COLOR_MAP_H_

#include "global_segment_map/common.h"

namespace voxblox {

class LabelColorMap {
 public:
  void getColor(const Label& label, Color* color);

 protected:
  std::map<Label, Color> color_map_;
};
}  // namespace voxblox

#endif
