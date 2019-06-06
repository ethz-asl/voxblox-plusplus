#ifndef GLOBAL_SEGMENT_MAP_UTILS_MESHING_UTILS_H_
#define GLOBAL_SEGMENT_MAP_UTILS_MESHING_UTILS_H_

namespace voxblox {
namespace utils {

inline void getColorFromNormals(const Point& normals, Color* color) {
  CHECK_NOTNULL(color);

  color->r = (normals.x() * 0.5f + 0.5f) * 255.0f;
  color->g = (normals.y() * 0.5f + 0.5f) * 255.0f;
  color->b = (normals.x() * 0.5f + 0.5f) * 255.0f;
  color->a = 255.0f;
}

inline void getColorFromLabelConfidence(const LabelVoxel& label_voxel,
                                        const LabelConfidence& max_confidence,
                                        Color* color) {
  CHECK_NOTNULL(color);
  *color = rainbowColorMap(label_voxel.label_confidence / max_confidence);
}

}  // namespace utils
}  // namespace voxblox

#endif
