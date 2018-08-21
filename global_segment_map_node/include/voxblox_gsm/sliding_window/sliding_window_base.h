#ifndef VOXBLOX_GSM_SLIDING_WINDOW_SLIDING_WINDOW_BASE_H
#define VOXBLOX_GSM_SLIDING_WINDOW_SLIDING_WINDOW_BASE_H

#include "voxblox_gsm/controller.h"

#include <tf/transform_broadcaster.h>

namespace voxblox {
namespace voxblox_gsm {
class SlidingWindowBase : public Controller {
 public:
  explicit SlidingWindowBase(ros::NodeHandle* node_handle);

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
      bool labels_list_is_complete = false) override;

 protected:
  /**
   * Calls the base class implementation to integrate point cloud segments into
   * the GSM. It then also checks whether a window update is necessary. If true,
   * the update is performed.
   * @param segment_point_cloud_msg
   */
  void segmentPointCloudCallback(
      const sensor_msgs::PointCloud2::Ptr& segment_point_cloud_msg) override;

  void checkAndUpdateWindow();
  void publishWindowContent();

  /*
   * Implements the decision whether a window update is necessary or not. I.e.
   * time based, spatial window...
   */
  virtual bool isWindowUpdateDue() = 0;

  /*
   * Used to determine whether a segment counts as inside or outside the sliding
   * window.
   */
  virtual bool isBlockWithinWindow(const BlockIndex& block_index) = 0;

  Transformation getCurrentCameraPose();

  void updateWindowContent();

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
                        modelify_msgs::GsmUpdate& gsm_update) override;

  /**
   * Calls the base method and then removes the elements from the list, which
   * were outside of the sliding window.
   * @param labels
   * @param get_all
   */
  void getLabelsToPublish(std::vector<Label>* labels, bool get_all) override;

  Transformation current_window_pose_;
  ros::Time current_window_timestamp_;
  std::vector<Label> removed_segments_;

  std::unordered_map<Label, LayerPair> label_to_layers_;
  std::vector<geometry_msgs::PoseStamped> window_trajectory_;
  std::string imu_frame_ = "imu";

  ros::Publisher trajectory_publisher_;
  ros::Time time_last_processed_segment_;
};
}  // namespace voxblox_gsm
}  // namespace voxblox

#endif  // VOXBLOX_GSM_SLIDING_WINDOW_SLIDING_WINDOW_BASE_H
