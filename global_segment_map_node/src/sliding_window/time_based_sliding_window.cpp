#include "voxblox_gsm/sliding_window/time_based_sliding_window.h"

namespace voxblox {
namespace voxblox_gsm {
TimeBasedSlidingWindow::TimeBasedSlidingWindow(ros::NodeHandle* node_handle)
    : SlidingWindowBase(node_handle) {
  node_handle_private_->param<float>("sliding_window/time_based/window_size_s",
                                     window_size_s_, window_size_s_);
  node_handle_private_->param<float>("sliding_window/time_based/overlap",
                                     window_overlap_, window_overlap_);
  update_period_ = window_size_s_ * (1.0f - window_overlap_);
}

bool TimeBasedSlidingWindow::isWindowUpdateDue() {
  return (time_last_processed_segment_ - current_window_timestamp_).toSec() >
         update_period_;
}

bool TimeBasedSlidingWindow::isBlockWithinWindow(
    const voxblox::BlockIndex& block_index) {
  Block<TsdfVoxel>::Ptr tsdf_block =
      map_->getTsdfLayerPtr()->getBlockPtrByIndex(block_index);
  const double time_since_last_update =
      tsdf_block->getTimePassedSinceLastUpdateNs() * 1e-9;
  return time_since_last_update < window_size_s_;
}

}  // namespace voxblox_gsm
}  // namespace voxblox
