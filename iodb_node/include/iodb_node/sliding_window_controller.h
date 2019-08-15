#ifndef VOXBLOX_GSM_SLIDING_WINDOW_CONTROLLER_H
#define VOXBLOX_GSM_SLIDING_WINDOW_CONTROLLER_H

#include <tf/transform_broadcaster.h>

#include <iodb_node/iodb_controller.h>

namespace voxblox {
namespace voxblox_gsm {
class SlidingWindowController : public IodbController {
 public:
  explicit SlidingWindowController(ros::NodeHandle* node_handle);

  /**
   * Checks whether the segments have already been extracted for the current
   * window. If so, it returns the already extracted ones. If not, the base
   * method is called and the segments are extracted.
   * @param labels
   * @param label_layers_map
   * @param labels_list_is_complete
   */
  void extractSegmentLayers(
      const std::vector<Label>& labels,
      std::unordered_map<Label, LayerPair>* label_layers_map,
      bool labels_list_is_complete = false);

 private:
  /**
   * Calls the base class implementation to integrate point cloud segments into
   * the GSM. It then also checks the TF using the checkTfCallback, which might
   * lead to an update of the sliding window.
   * @param segment_point_cloud_msg
   */
  void segmentPointCloudCallback(
      const sensor_msgs::PointCloud2::Ptr& segment_point_cloud_msg) override;
  /**
   * Sets the sliding window to the new position, removes segments which are
   * completely outide of its volume and publishes the remaining scene and
   * segments.
   * @param new_center center position of the sliding window
   */
  void updateAndPublishWindow(const Point& new_center);

  /**
   * Checks whether the camera has moved more than a
   * certain distance from the current window center. If so, the window is
   * updated.
   */
  void checkTfCallback();

  /**
   * Removes segments from the gsm which are outside the ball volume defined
   * by the radius and center.
   * @param radius
   * @param center
   */
  void removeSegmentsOutsideOfRadius(const FloatingPoint radius,
                                     const Point& center);

  /**
   * Publishes position of new sliding window. Useful for debugging or display.
   * @param position
   */
  void publishWindowTrajectory(const Point& position);

  /**
   * Calls base method and adds the position of the sliding window to the
   * message.
   * @param publisher
   * @param gsm_update
   */
  void publishGsmUpdate(const ros::Publisher& publisher,
                        modelify_msgs::GsmUpdate* gsm_update) override;

  /**
   * Calls the base method and then removes the elements from the list, which
   * were outside of the sliding window.
   * @param labels
   * @param get_all
   */
  void getLabelsToPublish(const bool get_all,
                          std::vector<Label>* labels) override;

  Transformation current_window_pose_;
  ros::Time current_window_timestamp_;

  std::unordered_map<Label, LayerPair> label_to_layers_;
  std::vector<Label> removed_segments_;
  FloatingPoint window_radius_ = 1.0f;
  FloatingPoint update_fraction_ = 0.5f;
  std::vector<geometry_msgs::PoseStamped> window_trajectory_;
  std::string imu_frame_ = "imu";

  ros::Publisher trajectory_publisher_;
  ros::Time time_last_processed_segment_;
};
}  // namespace voxblox_gsm
}  // namespace voxblox

#endif  // VOXBLOX_GSM_SLIDING_WINDOW_CONTROLLER_H
