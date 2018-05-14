#include "voxblox_gsm/sliding_window_controller.h"

#include <minkindr_conversions/kindr_tf.h>
#include <std_msgs/builtin_uint32.h>

namespace voxblox {
namespace voxblox_gsm {

SlidingWindowController::SlidingWindowController(ros::NodeHandle* node_handle)
    : Controller(node_handle) {
  double update_period = 1.0;
  node_handle_private_->param("sliding_window/tf_check_t", update_period,
                              update_period);
  ros::Duration period(update_period);
  tf_check_timer_ = node_handle_private_->createTimer(
      period, &SlidingWindowController::checkTfCallback, this);

  node_handle_private_->param<float>("sliding_window/radius", window_radius_,
                                     window_radius_);
  node_handle_private_->param<float>("sliding_window/update_fraction",
                                     update_fraction_, update_fraction_);
}

void SlidingWindowController::removeSegmentsOutsideOfRadius(float radius,
                                                            Point center) {
  removed_segments_.clear();
  std::vector<Label> all_labels = integrator_->getLabelsList();
  label_to_layers_.clear();
  constexpr bool kLabelsListIsComlete = true;
  extractSegmentLayers(all_labels, &label_to_layers_, kLabelsListIsComlete);

  for (const Label& label : all_labels) {
    auto it = label_to_layers_.find(label);
    LayerPair& layer_pair = it->second;

    // Iterate over all blocks of segment. If one of the blocks is inside the
    // window radius, the whole segment is valid. Otherwise, the segment is
    // removed from the gsm
    BlockIndexList blocks_of_label;
    layer_pair.first.getAllAllocatedBlocks(&blocks_of_label);
    bool has_block_within_radius = false;
    for (const BlockIndex& block_index : blocks_of_label) {
      Point center_block = getCenterPointFromGridIndex(
          block_index, map_config_.voxel_size * map_config_.voxels_per_side);
      double distance_x_y = sqrt(pow(center_block(0) - center(0), 2) +
                                 pow(center_block(1) - center(1), 2) +
                                 pow(center_block(2) - center(2), 2));
      if (distance_x_y < radius) {
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
  if (!label_to_layers_.empty()) {
    *label_layers_map = label_to_layers_;
  } else {
    Controller::extractSegmentLayers(labels, label_layers_map,
                                     labels_list_is_complete);
  }
}

void SlidingWindowController::checkTfCallback(const ros::TimerEvent& ev) {
  ros::Time start = ros::Time::now();
  ros::Duration diff = ev.current_real - ev.current_expected;
  LOG(WARNING) << "tf diff: " << diff.toSec() << "s";

  if (!received_first_message_) {
    return;
  }

  tf::StampedTransform tf_transform;
  getCurrentPosition(&tf_transform);

  constexpr double kTimeout = 20.0;
  ros::Duration time_since_last_update =
      tf_transform.stamp_ - current_window_position_.stamp_;

  tfScalar distance =
      tf_transform.getOrigin().distance(current_window_position_.getOrigin());
  LOG(WARNING) << "distance " << distance;

  if (distance > window_radius_ * update_fraction_ ||
      (time_since_last_update.toSec() > kTimeout &&
       window_has_moved_first_time_)) {
    window_has_moved_first_time_ = true;
    current_window_position_ = tf_transform;
    Transformation kindr_transform;
    tf::transformTFToKindr(tf_transform, &kindr_transform);
    current_window_position_point_ = kindr_transform.getPosition();
    updateAndPublishWindow(current_window_position_point_);
    publishPosition(current_window_position_point_);
  }
  LOG(WARNING) << "Done tf check";
  ros::Time stop = ros::Time::now();
  tf_check_time_ += stop - start;

  LOG(WARNING) << "total time tf check: " << tf_check_time_.toSec() << "s";
  LOG(WARNING) << "total time segments: " << segments_time_.toSec() << "s";
  LOG(WARNING) << "total time passed: " << (stop - node_start_time_).toSec() << "s";
}

void SlidingWindowController::updateAndPublishWindow(const Point& new_center) {
  LOG(WARNING) << "Update Window";
  removeSegmentsOutsideOfRadius(window_radius_, new_center);

  std_srvs::Empty::Request req;
  std_srvs::Empty::Response res;
  LOG(INFO) << "Publish scene";
  ros::Time start = ros::Time::now();
  publishSceneCallback(req, res);
  ros::Time stop = ros::Time::now();
  ros::Duration duration = stop - start;
  LOG(WARNING) << "Publishing took " << duration.toSec() << "s";
}

void SlidingWindowController::getCurrentPosition(
    tf::StampedTransform* position) {
  ros::Time time_now = ros::Time(0);
  try {
    tf_listener_.waitForTransform(
        world_frame_, camera_frame_, time_now,
        ros::Duration(30.0));  // in case rosbag has not been started yet
    tf_listener_.lookupTransform(world_frame_, camera_frame_, time_now,
                                 *position);
  } catch (tf::TransformException& ex) {
    LOG(FATAL) << "Error getting TF transform from sensor data: " << ex.what();
  }
}

void SlidingWindowController::publishGsmUpdate(
    const ros::Publisher& publisher, modelify_msgs::GsmUpdate& gsm_update) {
  geometry_msgs::Point point;
  point.x = current_window_position_point_(0);
  point.y = current_window_position_point_(1);
  point.z = current_window_position_point_(2);
  gsm_update.sliding_window_position = point;
  Controller::publishGsmUpdate(publisher, gsm_update);
}

void SlidingWindowController::publishPosition(const Point& position) {
  // Publish position.
  tf::StampedTransform tf;
  tf::Vector3 translation;
  translation.setX(position(0));
  translation.setY(position(1));
  translation.setZ(position(2));
  tf.setOrigin(translation);
  position_broadcaster_.sendTransform(
      tf::StampedTransform(tf, ros::Time::now(), "world", "sliding_window"));
}

void SlidingWindowController::getLabelsToPublish(std::vector<Label>* labels,
                                                 bool get_all) {
  Controller::getLabelsToPublish(labels, get_all);
  for (const Label& label : removed_segments_) {
    labels->erase(std::remove(labels->begin(), labels->end(), label));
  }
}

}  // namespace voxblox_gsm
}  // namespace voxblox
