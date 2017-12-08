#ifndef VOXBLOX_GSM_CONVERSIONS_H_
#define VOXBLOX_GSM_CONVERSIONS_H_

#include <vector>

#include <geometry_msgs/Transform.h>
#include <modelify_msgs/VoxelEvaluationDetails.h>
#include <voxblox/core/common.h>

namespace voxblox {
namespace voxblox_gsm {

inline void transformMsgs2Transformations(
    const std::vector<geometry_msgs::Transform>& transforms,
    std::vector<Transformation>* transformations) {
  CHECK_NOTNULL(transformations);
  typedef kindr::minimal::RotationQuaternionTemplate<voxblox::FloatingPoint>
      RotationQuaternion;
  typedef Eigen::Matrix<voxblox::FloatingPoint, 3, 1> Vector3;
  for (geometry_msgs::Transform transform : transforms) {
    RotationQuaternion quaternion(transform.rotation.w, transform.rotation.x,
                                  transform.rotation.y, transform.rotation.z);
    Vector3 translation(transform.translation.x, transform.translation.y,
                        transform.translation.z);
    transformations->emplace_back(quaternion, translation);
  }
}
inline void voxelEvaluationDetails2VoxelEvaluationDetailsMsg(
    const std::vector<voxblox::utils::VoxelEvaluationDetails>
        voxel_evaluation_details_vector,
    std::vector<modelify_msgs::VoxelEvaluationDetails>*
        voxel_evaluation_details_msgs) {
  CHECK_NOTNULL(voxel_evaluation_details_msgs);
  voxel_evaluation_details_msgs->clear();

  for (const voxblox::utils::VoxelEvaluationDetails voxel_evaluation_details :
       voxel_evaluation_details_vector) {
    // TODO(ff): Move this to voxblox_ros conversions.h.
    modelify_msgs::VoxelEvaluationDetails voxel_evaluation_details_msg;
    voxel_evaluation_details_msg.rmse = voxel_evaluation_details.rmse;
    voxel_evaluation_details_msg.max_error = voxel_evaluation_details.max_error;
    voxel_evaluation_details_msg.min_error = voxel_evaluation_details.min_error;
    voxel_evaluation_details_msg.num_evaluated_voxels =
        voxel_evaluation_details.num_evaluated_voxels;
    voxel_evaluation_details_msg.num_ignored_voxels =
        voxel_evaluation_details.num_ignored_voxels;
    voxel_evaluation_details_msg.num_overlapping_voxels =
        voxel_evaluation_details.num_overlapping_voxels;
    voxel_evaluation_details_msg.num_non_overlapping_voxels =
        voxel_evaluation_details.num_non_overlapping_voxels;
    voxel_evaluation_details_msgs->push_back(voxel_evaluation_details_msg);
  }
}
}  // namespace voxblox_gsm
}  // namespace voxblox
#endif  // VOXBLOX_GSM_CONVERSIONS_H_
