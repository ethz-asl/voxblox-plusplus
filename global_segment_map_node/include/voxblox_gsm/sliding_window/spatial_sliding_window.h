#ifndef VOXBLOX_GSM_SLIDING_WINDOW_SPATIAL_SLIDING_WINDOW_H
#define VOXBLOX_GSM_SLIDING_WINDOW_SPATIAL_SLIDING_WINDOW_H

#include "voxblox_gsm/sliding_window/sliding_window_base.h"

namespace voxblox {
namespace voxblox_gsm {

class SpatialSlidingWindow : public SlidingWindowBase {
public:
  explicit SpatialSlidingWindow(ros::NodeHandle* node_handle);

private:
  bool isWindowUpdateDue() override;
  /*
   * Checks whether the center of a block is within the radius of the spatial
   * sliding window.
   */
  bool isBlockWithinWindow(const BlockIndex& block_index) override;

  FloatingPoint window_radius_ = 1.0f;
  FloatingPoint update_fraction_ = 0.5f;
};

} // namespace voxblox_gsm
} // namespace voxblox

#endif //VOXBLOX_GSM_NODE_SLIDING_WINDOW_SPATIAL_SLIDING_WINDOW_H
