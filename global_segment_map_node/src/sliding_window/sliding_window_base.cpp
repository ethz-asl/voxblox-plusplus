#include "voxblox_gsm/sliding_window/sliding_window_base.h"

#include <minkindr_conversions/kindr_tf.h>
#include <nav_msgs/Path.h>
#include <std_msgs/builtin_uint32.h>

namespace voxblox {
namespace voxblox_gsm {

SlidingWindowBase::SlidingWindowBase(ros::NodeHandle* node_handle)
    : Controller(node_handle) {
  node_handle_private_->param<std::string>("sliding_window/imu_frame",
                                           imu_frame_, imu_frame_);

  trajectory_publisher_ =
      node_handle_private_->advertise<nav_msgs::Path>("window_trajectory", 200);
}

void SlidingWindowBase::extractSegmentLayers(
    const std::vector<Label>& labels,
    std::unordered_map<Label, LayerPair>* label_layers_map,
    bool labels_list_is_complete) {
  if (!label_to_layers_.empty()) {
    *label_layers_map = label_to_layers_;
  } else {
    Controller::extractSegmentLayers(labels, label_layers_map,
                                     labels_list_is_complete);
  }
}

void SlidingWindowBase::checkAndUpdateWindow() {
  if (isWindowUpdateDue()) {
    current_window_pose_ = getCurrentCameraPose();
    current_window_timestamp_ = time_last_processed_segment_;

    LOG(INFO) << "Update Window";
    updateWindowContent();
    publishWindowContent();
    publishWindowTrajectory(current_window_pose_.getPosition());
  }
}

void SlidingWindowBase::updateWindowContent() {
  removed_segments_.clear();
  std::vector<Label> all_labels = integrator_->getLabelsList();
  label_to_layers_.clear();
  constexpr bool kLabelsListIsComlete = true;
  extractSegmentLayers(all_labels, &label_to_layers_, kLabelsListIsComlete);

  for (const Label& label : all_labels) {
    auto it = label_to_layers_.find(label);
    LayerPair& layer_pair = it->second;

    // Iterate over all blocks of segment. If one of the blocks is inside the
    // window, the whole segment is valid. Otherwise, the blocks
    // containing the segment are removed from the GSM
    BlockIndexList blocks_of_label;
    layer_pair.first.getAllAllocatedBlocks(&blocks_of_label);
    bool has_block_within_window = false;
    for (const BlockIndex& block_index : blocks_of_label) {
      has_block_within_window = isBlockWithinWindow(block_index);
      if (has_block_within_window) {
        break;
      }
    }
    if (!has_block_within_window) {
      for (const BlockIndex& block_index : blocks_of_label) {
        map_->getTsdfLayerPtr()->removeBlock(block_index);
        map_->getLabelLayerPtr()->removeBlock(block_index);
      }
      label_to_layers_.erase(label);
      removed_segments_.push_back(label);
    }
  }
}

void SlidingWindowBase::publishWindowContent() {
  LOG(INFO) << "Publish scene";
  std_srvs::SetBool::Request req;
  constexpr bool kCreateMesh = false;
  req.data = kCreateMesh;
  std_srvs::SetBool::Response res;
  ros::Time start = ros::Time::now();
  publishSceneCallback(req, res);
  ros::Time stop = ros::Time::now();
  LOG(WARNING) << "Publishing took " << (stop - start).toSec() << "s";
}

void SlidingWindowBase::publishGsmUpdate(const ros::Publisher& publisher,
                                         modelify_msgs::GsmUpdate& gsm_update) {
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

  gsm_update.sliding_window_pose = pose;
  Controller::publishGsmUpdate(publisher, gsm_update);
}

void SlidingWindowBase::publishWindowTrajectory(const Point& position) {
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

void SlidingWindowBase::getLabelsToPublish(std::vector<Label>* labels,
                                           bool get_all) {
  Controller::getLabelsToPublish(labels, get_all);
  for (const Label& label : removed_segments_) {
    labels->erase(std::remove(labels->begin(), labels->end(), label));
  }
}

void SlidingWindowBase::segmentPointCloudCallback(
    const sensor_msgs::PointCloud2::Ptr& segment_point_cloud_msg) {
  Controller::segmentPointCloudCallback(segment_point_cloud_msg);

  // update sliding window if necessary
  const ros::Time& current_stamp = segment_point_cloud_msg->header.stamp;
  if (current_stamp != time_last_processed_segment_) {
    checkAndUpdateWindow();
    time_last_processed_segment_ = current_stamp;
  }
}

Transformation SlidingWindowBase::getCurrentCameraPose() {
  Transformation camera_pose;
  lookupTransform(imu_frame_, world_frame_, time_last_processed_segment_,
                  &camera_pose);
  return camera_pose;
}

}  // namespace voxblox_gsm
}  // namespace voxblox
