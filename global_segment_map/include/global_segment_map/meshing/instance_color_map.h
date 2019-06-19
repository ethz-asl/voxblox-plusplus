#ifndef GLOBAL_SEGMENT_MAP_MESHING_INSTANCE_COLOR_MAP_H_
#define GLOBAL_SEGMENT_MAP_MESHING_INSTANCE_COLOR_MAP_H_

#include "global_segment_map/common.h"

namespace voxblox {

class InstanceColorMap {
 public:
  void getColor(const InstanceLabel& instance_label, Color* color);

 protected:
  std::map<InstanceLabel, Color> color_map_;
};
}  // namespace voxblox

#endif  // GLOBAL_SEGMENT_MAP_MESHING_INSTANCE_COLOR_MAP_H_
