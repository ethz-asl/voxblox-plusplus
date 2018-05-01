#ifndef VOXBLOX_GSM_SLIDING_WINDOW_CONTROLLER_H
#define VOXBLOX_GSM_SLIDING_WINDOW_CONTROLLER_H

#include "controller.h"

namespace voxblox {
namespace voxblox_gsm {
class SlidingWindowController : public Controller {
 public:
  SlidingWindowController(ros::NodeHandle* node_handle);

  void updateWindowCallback(const ros::TimerEvent&);

  void extractAllSegmentLayers(
      const std::vector<Label>& labels,
      std::unordered_map<Label, LayerPair>* label_layers_map,
      bool labels_list_is_complete = true) override;

  void advertiseSegmentRemovalTopic();

 private:
  void removeSegmentsOutsideOfRadius(float radius);
  std::unordered_map<Label, LayerPair> label_to_layers_;
  ros::Publisher removed_segments_pub_;
  ros::Timer timer_;
  ros::ServiceClient new_window_client_;
};
}  // namespace voxblox_gsm
}  // namespace voxblox

#endif  // VOXBLOX_GSM_SLIDING_WINDOW_CONTROLLER_H
