#ifndef VOXBLOX_GSM_SLIDING_WINDOW_CONTROLLER_H
#define VOXBLOX_GSM_SLIDING_WINDOW_CONTROLLER_H

#include "controller.h"

#include <tf/transform_broadcaster.h>

namespace voxblox {
namespace voxblox_gsm {
class SlidingWindowController : public Controller {
 public:
  explicit SlidingWindowController(ros::NodeHandle* node_handle);

  void extractAllSegmentLayers(
      const std::vector<Label>& labels,
      std::unordered_map<Label, LayerPair>* label_layers_map,
      bool labels_list_is_complete = true) override;

 private:
  void updateAndPublishWindow(const Point& new_center);

  void checkTfCallback(const ros::TimerEvent&);

  void removeSegmentsOutsideOfRadius(float radius, Point center);

  void getCurrentPosition(tf::StampedTransform* position);

  void publishPosition(const Point& position);

  void publishGsmUpdate(const ros::Publisher& publisher,
                        modelify_msgs::GsmUpdate& gsm_update) override;

  void getLabelsToPublish(std::vector<Label>* labels, bool get_all) override;

  ros::Timer tf_check_timer_;
  tf::TransformBroadcaster position_broadcaster_;
  tf::StampedTransform current_window_position_;

  std::unordered_map<Label, LayerPair> label_to_layers_;
  Point current_window_position_point_;
  std::vector<Label> removed_segments_;
  float window_radius_ = 1.0f;
  bool window_has_moved_first_time_= false;
};
}  // namespace voxblox_gsm
}  // namespace voxblox

#endif  // VOXBLOX_GSM_SLIDING_WINDOW_CONTROLLER_H
