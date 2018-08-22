#include "voxblox_gsm/sliding_window/spatial_sliding_window.h"

namespace voxblox {
namespace voxblox_gsm {

SpatialSlidingWindow::SpatialSlidingWindow(
    ros::NodeHandle *node_handle) : SlidingWindowBase(node_handle) {
  node_handle_private_->param<float>("sliding_window/spatial/radius_m", window_radius_,
                                     window_radius_);
  node_handle_private_->param<float>("sliding_window/spatial/update_fraction",
                                     update_fraction_, update_fraction_);

}

bool SpatialSlidingWindow::isWindowUpdateDue() {
  Transformation camera_pose = getCurrentCameraPose();
  const float distance =
      (camera_pose.getPosition() - current_window_pose_.getPosition()).norm();
  return distance > window_radius_ * update_fraction_;
}

bool SpatialSlidingWindow::isBlockWithinWindow(const BlockIndex& block_index) {
const Point center_block = getCenterPointFromGridIndex(
    block_index, map_config_.voxel_size * map_config_.voxels_per_side);
const Point center = current_window_pose_.getPosition();
const double distance_x_y = sqrt(pow(center_block(0) - center(0), 2) +
                           pow(center_block(1) - center(1), 2) +
                           pow(center_block(2) - center(2), 2));
return distance_x_y < window_radius_;
}

} // namespace voxblox_gsm
} // namespace voxblox
