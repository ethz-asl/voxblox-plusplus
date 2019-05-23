#ifndef VOXBLOX_GSM_FEATURE_ROS_TOOLS_H
#define VOXBLOX_GSM_FEATURE_ROS_TOOLS_H
// TODO(ntonci): These include guards have wrong format.
// TODO(ntonci): We should probably rename this package.
// TODO(ntonci): We should probably rename the repo.

#include <algorithm>

#include <modelify_msgs/Features.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <voxblox/core/common.h>

#include <global_feature_map/feature_block.h>
#include <global_feature_map/feature_layer.h>
#include <global_feature_map/feature_types.h>

namespace voxblox {
namespace voxblox_gsm {

std_msgs::ColorRGBA getColorFromBlockFeatures(
    const double max_number_of_features, const double number_of_features);

template <typename FeatureType>
void createOccupancyBlocksFromFeatureLayer(
    const FeatureLayer<FeatureType>& layer, const std::string& frame_id,
    const double& max_number_of_features,
    visualization_msgs::MarkerArray* marker_array) {
  CHECK_NOTNULL(marker_array);

  // Cache layer settings.
  FloatingPoint block_size = layer.block_size();

  visualization_msgs::Marker block_marker;
  block_marker.header.frame_id = frame_id;
  block_marker.ns = "feature_blocks";
  block_marker.id = 0;
  block_marker.type = visualization_msgs::Marker::CUBE_LIST;
  block_marker.scale.x = block_marker.scale.y = block_marker.scale.z =
      block_size;
  block_marker.action = visualization_msgs::Marker::ADD;

  BlockIndexList blocks;
  layer.getAllAllocatedBlocks(&blocks);
  for (const BlockIndex& index : blocks) {
    const FeatureBlock<FeatureType>& block = layer.getBlockByIndex(index);
    Point coord = block.origin();
    geometry_msgs::Point cube_center;
    cube_center.x = coord.x() + block_size / 2.0;
    cube_center.y = coord.y() + block_size / 2.0;
    cube_center.z = coord.z() + block_size / 2.0;
    block_marker.points.push_back(cube_center);
    size_t number_of_features = block.numFeatures();
    std_msgs::ColorRGBA color_msg =
        getColorFromBlockFeatures(max_number_of_features, number_of_features);
    block_marker.colors.push_back(color_msg);
  }
  marker_array->markers.push_back(block_marker);
}

void fromFeaturesMsgToFeature3D(const modelify_msgs::Features& features_msg,
                                size_t* descriptor_size,
                                std::string* camera_frame, ros::Time* timestamp,
                                std::vector<Feature3D>* features_C);

}  // namespace voxblox_gsm
}  // namespace voxblox

#endif  // VOXBLOX_GSM_FEATURE_ROS_TOOLS_H
