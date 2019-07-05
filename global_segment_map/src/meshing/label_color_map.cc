#include "global_segment_map/meshing/label_color_map.h"

#include "voxblox/core/color.h"

namespace voxblox {

void LabelColorMap::getColor(const Label& label, Color* color) {
  CHECK_NOTNULL(color);
  CHECK_NE(label, 0u);

  std::map<Label, Color>::iterator label_color_map_it;
  {
    std::shared_lock<std::shared_timed_mutex> readerLock(color_map_mutex_);
    label_color_map_it = color_map_.find(label);
  }

  if (label_color_map_it != color_map_.end()) {
    *color = label_color_map_it->second;
  } else {
    *color = randomColor();

    std::lock_guard<std::shared_timed_mutex> writerLock(color_map_mutex_);
    color_map_.insert(std::pair<Label, Color>(label, *color));
  }
}
}  // namespace voxblox
