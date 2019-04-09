#ifndef GLOBAL_SEGMENT_MAP_UTILS_COLOR_MAP_H_
#define GLOBAL_SEGMENT_MAP_UTILS_COLOR_MAP_H_

#include <voxblox/core/color.h>

#include "global_segment_map/common.h"

namespace voxblox {

class SemanticColorMap {
 public:
  enum SemanticColor { kCoco, kNyuV2_13 };

  static SemanticColorMap create(SemanticColor color_mode);

  SemanticColorMap(const std::vector<std::array<float, 3>> color_code)
      : color_code_(color_code) {}

  Color getColor(SemanticLabel semantic_label) const {
    return Color(color_code_.at(semantic_label)[0],
                 color_code_.at(semantic_label)[1],
                 color_code_.at(semantic_label)[2]);
  }

 protected:
  std::vector<std::array<float, 3>> color_code_;
};

// NYUv2 13 class task color coding defined in SceneNet.
class NyuV213ColorMap : public SemanticColorMap {
 public:
  NyuV213ColorMap();
};

// COCO 80 class task color coding using the PASCAL VOC color map.
class CocoColorMap : public SemanticColorMap {
 public:
  CocoColorMap();
};
}  // namespace voxblox

#endif  // GLOBAL_SEGMENT_MAP_UTILS_COLOR_MAP_H_
