#include "global_segment_map/meshing/label_color_map.h"

#include "voxblox/core/color.h"

namespace voxblox {

void LabelColorMap::getColor(const Label& label, Color* color) {
  DCHECK(color != nullptr);
  CHECK_NE(label, 0u);

  auto label_color_map_it = color_map_.find(label);

  if (label_color_map_it != color_map_.end()) {
    *color = label_color_map_it->second;
  } else {
    *color = randomColor();

    color_map_.insert(std::pair<Label, Color>(label, *color));
  }
}
}  // namespace voxblox
