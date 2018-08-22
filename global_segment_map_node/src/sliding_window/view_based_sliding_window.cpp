#include "voxblox_gsm/sliding_window/view_based_sliding_window.h"

namespace voxblox {
namespace voxblox_gsm {

ViewBasedSlidingWindow::ViewBasedSlidingWindow(ros::NodeHandle* node_handle)
    : SlidingWindowBase(node_handle) {
  FloatingPoint view_angle_deg = rad2deg(view_angle_rad_);
  node_handle_private_->param<FloatingPoint>(
      "sliding_window/view_based/view_angle_deg", view_angle_deg,
      view_angle_deg);
  view_angle_rad_ = deg2rad(view_angle_deg);

  FloatingPoint update_angle_deg = rad2deg(update_angle_rad_);
  node_handle_private_->param<FloatingPoint>(
      "sliding_window/view_based/update_angle_deg", update_angle_deg,
      update_angle_deg);
  update_angle_rad_ = static_cast<float>(fmod(deg2rad(update_angle_deg), M_PI));

  node_handle_private_->param<FloatingPoint>(
      "sliding_window/view_based/update_distance_m", update_distance_m_,
      update_distance_m_);
}

bool ViewBasedSlidingWindow::isWindowUpdateDue() {
  const Transformation current_camera_pose = getCurrentCameraPose();
  const FloatingPoint distance =
      (current_window_pose_.getPosition() - current_camera_pose.getPosition())
          .norm();

  const Point camera_axis_window =
      current_window_pose_.getRotationMatrix().block<3, 1>(0, 3);
  const Point camera_axis_current =
      current_camera_pose.getRotationMatrix().block<3, 1>(0, 3);
  const FloatingPoint angle_difference =
      angleRadFromTwoVectors(camera_axis_current, camera_axis_window);

  return distance > update_distance_m_ || angle_difference > update_angle_rad_;

}

bool ViewBasedSlidingWindow::isBlockWithinWindow(
    const BlockIndex& block_index) {

}

FloatingPoint ViewBasedSlidingWindow::angleRadFromTwoVectors(Point a, Point b) {
  // <a,b> = ||a||*||b||*cos(a,b)
  const FloatingPoint norm_a = a.norm();
  const FloatingPoint norm_b = a.norm();
  const FloatingPoint a_times_b = a.dot(b);

  return std::acos(a_times_b / (norm_a * norm_b));
}

}  // namespace voxblox_gsm
}  // namespace voxblox
