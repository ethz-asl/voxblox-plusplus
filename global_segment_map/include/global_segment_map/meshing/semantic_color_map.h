#ifndef GLOBAL_SEGMENT_MAP_MESHING_SEMANTIC_COLOR_MAP_H_
#define GLOBAL_SEGMENT_MAP_MESHING_SEMANTIC_COLOR_MAP_H_

#include <array>
#include <vector>

#include <voxblox/core/color.h>

#include "global_segment_map/common.h"

namespace voxblox {

class SemanticColorMap {
 public:
  enum ClassTask { kCoco80 = 0, kNyu13 };

  static SemanticColorMap create(const ClassTask& class_task);

  SemanticColorMap(const std::vector<std::array<float, 3>>& color_code)
      : color_code_(color_code) {}

  void getColor(const SemanticLabel& semantic_label, Color* color) const;

 protected:
  std::vector<std::array<float, 3>> color_code_;
};

// NYUv2 13 class task color coding defined in SceneNet.
class Nyu13ColorMap : public SemanticColorMap {
 public:
  Nyu13ColorMap();
};

// COCO 80 class task color coding using the PASCAL VOC color map.
class CocoColorMap : public SemanticColorMap {
 public:
  CocoColorMap();
};
}  // namespace voxblox

#endif  // GLOBAL_SEGMENT_MAP_MESHING_SEMANTIC_COLOR_MAP_H_
