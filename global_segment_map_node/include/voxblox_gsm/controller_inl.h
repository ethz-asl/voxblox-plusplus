// Copyright 2018 Margarita Grinvald, ASL, ETH Zurich, Switzerland

#ifndef VOXBLOX_GSM_INCLUDE_VOXBLOX_GSM_CONTROLLER_INL_H_
#define VOXBLOX_GSM_INCLUDE_VOXBLOX_GSM_CONTROLLER_INL_H_

#include <pcl_conversions/pcl_conversions.h>

namespace voxblox {
namespace voxblox_gsm {

template <typename point_type>
void Controller::fillSegmentWithData(
    const sensor_msgs::PointCloud2::Ptr& segment_point_cloud_msg,
    Segment* segment) {
  pcl::PointCloud<point_type> point_cloud;

  pcl::fromROSMsg(*segment_point_cloud_msg, point_cloud);

  segment->points_C_.reserve(point_cloud.points.size());
  segment->colors_.reserve(point_cloud.points.size());

  for (size_t i = 0; i < point_cloud.points.size(); ++i) {
    if (!std::isfinite(point_cloud.points[i].x) ||
        !std::isfinite(point_cloud.points[i].y) ||
        !std::isfinite(point_cloud.points[i].z)) {
      continue;
    }

    segment->points_C_.push_back(Point(point_cloud.points[i].x,
                                       point_cloud.points[i].y,
                                       point_cloud.points[i].z));

    segment->colors_.push_back(
        Color(point_cloud.points[i].r, point_cloud.points[i].g,
              point_cloud.points[i].b, point_cloud.points[i].a));
  }
}
}  // namespace voxblox_gsm
}  // namespace voxblox

#endif  // VOXBLOX_GSM_INCLUDE_VOXBLOX_GSM_CONTROLLER_INL_H_
