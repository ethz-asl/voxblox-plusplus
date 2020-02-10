#ifndef GLOBAL_SEGMENT_MAP_UTILS_ICP_H_
#define GLOBAL_SEGMENT_MAP_UTILS_ICP_H_

#include <pcl/registration/icp.h>

namespace voxblox {

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloudType;

class ICP {
  pcl::IterativeClosestPoint<PointType, PointType> icp_;

  int max_iterations_;

 public:
  ICP();

  void align(const PointCloudType::Ptr cloud_in,
             const PointCloudType::Ptr cloud_tr,
             Eigen::Matrix4f* transformation_matrix);
};

}  // namespace voxblox

#endif  // GLOBAL_SEGMENT_MAP_UTILS_ICP_UTILS_H_
