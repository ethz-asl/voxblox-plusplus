#ifndef VOXBLOX_GSM_SLIDING_WINDOW_TIME_BASED_SLIDING_WINDOW_H
#define VOXBLOX_GSM_SLIDING_WINDOW_TIME_BASED_SLIDING_WINDOW_H

#include "voxblox_gsm/sliding_window/sliding_window_base.h"

namespace voxblox {
namespace voxblox_gsm {
class TimeBasedSlidingWindow : public SlidingWindowBase {
public:
  explicit TimeBasedSlidingWindow(ros::NodeHandle* node_handle);

private:
  bool isWindowUpdateDue() override;
  bool isBlockWithinWindow(const BlockIndex& block_index) override;

  FloatingPoint window_size_s_= 5.0f;
  FloatingPoint window_overlap_ = 0.5f;
  FloatingPoint update_period_;
};

} // namespace voxblox_gsm
} // namespace voxblox

#endif //VOXBLOX_GSM_SLIDING_WINDOW_TIME_BASED_SLIDING_WINDOW_H
