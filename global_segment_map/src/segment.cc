#include "global_segment_map/segment.h"
namespace voxblox {

Segment::Segment(const pcl::PointCloud<voxblox::PointType>& point_cloud,
                 const Transformation& T_G_C)
    : T_G_C_(T_G_C), semantic_label_(0u), instance_label_(0u) {
  points_C_.reserve(point_cloud.points.size());
  colors_.reserve(point_cloud.points.size());

  for (size_t i = 0; i < point_cloud.points.size(); ++i) {
    if (!std::isfinite(point_cloud.points[i].x) ||
        !std::isfinite(point_cloud.points[i].y) ||
        !std::isfinite(point_cloud.points[i].z)) {
      continue;
    }

    points_C_.push_back(Point(point_cloud.points[i].x, point_cloud.points[i].y,
                              point_cloud.points[i].z));

    colors_.push_back(Color(point_cloud.points[i].r, point_cloud.points[i].g,
                            point_cloud.points[i].b, point_cloud.points[i].a));
  }
}

Segment::Segment(const pcl::PointCloud<voxblox::PointLabelType>& point_cloud,
                 const Transformation& T_G_C)
    : T_G_C_(T_G_C), label_(point_cloud.points[0].label) {
  points_C_.reserve(point_cloud.points.size());
  colors_.reserve(point_cloud.points.size());

  for (size_t i = 0u; i < point_cloud.points.size(); ++i) {
    if (!std::isfinite(point_cloud.points[i].x) ||
        !std::isfinite(point_cloud.points[i].y) ||
        !std::isfinite(point_cloud.points[i].z)) {
      continue;
    }

    points_C_.push_back(Point(point_cloud.points[i].x, point_cloud.points[i].y,
                              point_cloud.points[i].z));

    colors_.push_back(Color(point_cloud.points[i].r, point_cloud.points[i].g,
                            point_cloud.points[i].b, point_cloud.points[i].a));
  }
}

Segment::Segment(
    const pcl::PointCloud<voxblox::PointSemanticInstanceType>& point_cloud,
    const Transformation& T_G_C)
    : T_G_C_(T_G_C),
      semantic_label_(point_cloud.points[0].semantic_label),
      instance_label_(point_cloud.points[0].instance_label) {
  points_C_.reserve(point_cloud.points.size());
  colors_.reserve(point_cloud.points.size());

  for (size_t i = 0u; i < point_cloud.points.size(); ++i) {
    if (!std::isfinite(point_cloud.points[i].x) ||
        !std::isfinite(point_cloud.points[i].y) ||
        !std::isfinite(point_cloud.points[i].z)) {
      continue;
    }

    points_C_.push_back(Point(point_cloud.points[i].x, point_cloud.points[i].y,
                              point_cloud.points[i].z));

    colors_.push_back(Color(point_cloud.points[i].r, point_cloud.points[i].g,
                            point_cloud.points[i].b, point_cloud.points[i].a));
  }
}

}  // namespace voxblox
