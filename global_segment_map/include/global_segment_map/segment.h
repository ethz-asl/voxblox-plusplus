#ifndef GLOBAL_SEGMENT_MAP_SEGMENT_H_
#define GLOBAL_SEGMENT_MAP_SEGMENT_H_

#include <global_segment_map/common.h>

namespace voxblox {

class Segment {
 public:
  Segment(const pcl::PointCloud<voxblox::PointType>& point_cloud,
          const Transformation& T_G_C);

  Segment(const pcl::PointCloud<voxblox::PointLabelType>& point_cloud,
          const Transformation& T_G_C);

  Segment(
      const pcl::PointCloud<voxblox::PointSemanticInstanceType>& point_cloud,
      const Transformation& T_G_C);

  voxblox::Transformation T_G_C_;
  voxblox::Pointcloud points_C_;
  voxblox::Colors colors_;
  voxblox::Label label_;
  voxblox::SemanticLabel semantic_label_;
  voxblox::InstanceLabel instance_label_;
};
}  // namespace voxblox

#endif  // GLOBAL_SEGMENT_MAP_SEGMENT_H_
