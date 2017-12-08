#ifndef VOXBLOX_GSM_CONVERSIONS_H_
#define VOXBLOX_GSM_CONVERSIONS_H_

#include <vector>

#include <geometry_msgs/Transform.h>
#include <voxblox/core/common.h>

namespace voxblox {
namespace voxblox_gsm {

inline void transformMsgs2Transformations(
    const std::vector<geometry_msgs::Transform>& transforms,
    std::vector<Transformation>* transformations) {
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

}  // namespace voxblox_gsm
}  // namespace voxblox
#endif  // VOXBLOX_GSM_CONVERSIONS_H_
