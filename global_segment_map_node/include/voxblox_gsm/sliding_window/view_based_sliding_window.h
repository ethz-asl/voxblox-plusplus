#ifndef VOXBLOX_GSM_SLIDING_WINDOW_VIEW_BASED_SLIDING_WINDOW_H
#define VOXBLOX_GSM_SLIDING_WINDOW_VIEW_BASED_SLIDING_WINDOW_H

#include "voxblox_gsm/sliding_window/sliding_window_base.h"

namespace voxblox {
namespace voxblox_gsm {

class ViewBasedSlidingWindow : public SlidingWindowBase {
public:
  explicit ViewBasedSlidingWindow(ros::NodeHandle* node_handle);

private:
  bool isWindowUpdateDue() override;
  bool isBlockWithinWindow(const BlockIndex& block_index) override;

  FloatingPoint angleRadFromTwoVectors(Point a, Point b);

  template<typename T>
  T deg2rad(T degrees) {
    return degrees / static_cast<T>(180) * static_cast<T>(M_PI);
  }
  template<typename T>
  T rad2deg(T radians) {
    return radians / static_cast<T>(M_PI) * static_cast<T>(180) ;
  }

  FloatingPoint view_angle_rad_= 3.14f;
  FloatingPoint update_distance_m_ = 3.14f;
  FloatingPoint update_angle_rad_= 3.14f;
};

} // namespace voxblox_gsm
} // namespace voxblox

#endif //VOXBLOX_GSM_SLIDING_WINDOW_VIEW_BASED_SLIDING_WINDOW_H
