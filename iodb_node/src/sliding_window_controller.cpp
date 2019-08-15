#include "iodb_node/sliding_window_controller.h"

#include <minkindr_conversions/kindr_tf.h>
#include <nav_msgs/Path.h>
#include <std_msgs/builtin_uint32.h>

namespace voxblox {
namespace voxblox_gsm {

SlidingWindowController::SlidingWindowController(ros::NodeHandle* node_handle)
    : IodbController(CHECK_NOTNULL(node_handle)) {
  node_handle_private_->param<float>("sliding_window/radius_m", window_radius_,
                                     window_radius_);
  node_handle_private_->param<float>("sliding_window/update_fraction",
                                     update_fraction_, update_fraction_);
  node_handle_private_->param<std::string>("imu_frame_id", imu_frame_,
                                           imu_frame_);

  trajectory_publisher_ =
      node_handle_private_->advertise<nav_msgs::Path>("window_trajectory", 200);
}

void SlidingWindowController::removeSegmentsOutsideOfRadius(
    const float radius, const Point& center) {
  removed_segments_.clear();
  std::vector<Label> all_labels = map_->getLabelList();
  label_to_layers_.clear();
  constexpr bool kLabelsListIsComlete = true;
  extractSegmentLayers(all_labels, &label_to_layers_, kLabelsListIsComlete);

  for (const Label& label : all_labels) {
    auto it = label_to_layers_.find(label);
    LayerPair& layer_pair = it->second;

    // Iterate over all blocks of segment. If one of the blocks is inside the
    // window radius, the whole segment is valid. Otherwise, the blocks
    // containing the segment are removed from the GSM
    BlockIndexList blocks_of_label;
    layer_pair.first.getAllAllocatedBlocks(&blocks_of_label);
    bool has_block_within_radius = false;
    for (const BlockIndex& block_index : blocks_of_label) {
      Point center_block = getCenterPointFromGridIndex(
          block_index, map_config_.voxel_size * map_config_.voxels_per_side);

      const double distance_to_center = (center_block - center).norm();
      if (distance_to_center < radius) {
        has_block_within_radius = true;
        break;
      }
    }
    if (!has_block_within_radius) {
      for (const BlockIndex& block_index : blocks_of_label) {
        map_->getTsdfLayerPtr()->removeBlock(block_index);
        map_->getLabelLayerPtr()->removeBlock(block_index);
      }
      label_to_layers_.erase(label);
      removed_segments_.push_back(label);
    }
  }
}

void SlidingWindowController::extractSegmentLayers(
    const std::vector<Label>& labels,
    std::unordered_map<Label, LayerPair>* label_layers_map,
    bool labels_list_is_complete) {
  CHECK_NOTNULL(label_layers_map);
  if (!label_to_layers_.empty()) {
    *label_layers_map = label_to_layers_;
  } else {
    map_->extractSegmentLayers(labels, label_layers_map,
                               labels_list_is_complete);
  }
}

void SlidingWindowController::checkTfCallback() {
  if (!received_first_message_) {
    return;
  }

  // todo get pose timestamp
  Transformation camera_pose;
  lookupTransform(imu_frame_, world_frame_, time_last_processed_segment_,
                  &camera_pose);

  const float distance =
      (camera_pose.getPosition() - current_window_pose_.getPosition()).norm();
  LOG(INFO) << "Distance between camera and center of sliding window: "
            << distance;

  if (distance > window_radius_ * update_fraction_) {
    current_window_pose_ = camera_pose;
    current_window_timestamp_ = time_last_processed_segment_;

    updateAndPublishWindow(current_window_pose_.getPosition());
    publishWindowTrajectory(current_window_pose_.getPosition());
  }
}

void SlidingWindowController::updateAndPublishWindow(const Point& new_center) {
  LOG(INFO) << "Update Window";
  removeSegmentsOutsideOfRadius(window_radius_, new_center);

  LOG(INFO) << "Publish scene";
  std_srvs::SetBool::Request req;
  constexpr bool kCreateMesh = false;
  req.data = kCreateMesh;
  std_srvs::SetBool::Response res;
  ros::Time start = ros::Time::now();
  publishSceneCallback(req, res);
  ros::Time stop = ros::Time::now();
  LOG(INFO) << "Publishing took " << (stop - start).toSec() << "s";
}

void SlidingWindowController::publishGsmUpdate(
    const ros::Publisher& publisher, modelify_msgs::GsmUpdate* gsm_update) {
  CHECK_NOTNULL(gsm_update);
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = current_window_pose_.getPosition()(0);
  pose.pose.position.y = current_window_pose_.getPosition()(1);
  pose.pose.position.z = current_window_pose_.getPosition()(2);
  pose.pose.orientation.x = current_window_pose_.getRotation().x();
  pose.pose.orientation.y = current_window_pose_.getRotation().y();
  pose.pose.orientation.z = current_window_pose_.getRotation().z();
  pose.pose.orientation.w = current_window_pose_.getRotation().w();
  pose.header.frame_id = "world";
  pose.header.stamp = current_window_timestamp_;

  gsm_update->sliding_window_pose = pose;
  IodbController::publishGsmUpdate(publisher, gsm_update);
}

void SlidingWindowController::publishWindowTrajectory(const Point& position) {
  // Publish position.
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = world_frame_;
  pose.pose.position.x = position(0);
  pose.pose.position.y = position(1);
  pose.pose.position.z = position(2);
  window_trajectory_.push_back(pose);

  nav_msgs::Path msg;
  msg.poses = window_trajectory_;
  msg.header.frame_id = world_frame_;
  trajectory_publisher_.publish(msg);
}

void SlidingWindowController::getLabelsToPublish(const bool get_all,
                                                 std::vector<Label>* labels) {
  CHECK_NOTNULL(labels);
  IodbController::getLabelsToPublish(get_all, labels);
  for (const Label& label : removed_segments_) {
    labels->erase(std::remove(labels->begin(), labels->end(), label));
  }
}

void SlidingWindowController::segmentPointCloudCallback(
    const sensor_msgs::PointCloud2::Ptr& segment_point_cloud_msg) {
  Controller::segmentPointCloudCallback(segment_point_cloud_msg);

  // update sliding window if necessary
  const ros::Time& current_stamp = segment_point_cloud_msg->header.stamp;
  if (current_stamp != time_last_processed_segment_) {
    checkTfCallback();
    time_last_processed_segment_ = current_stamp;
  }
}

}  // namespace voxblox_gsm
}  // namespace voxblox
