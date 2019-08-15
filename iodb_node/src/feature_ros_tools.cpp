#include "iodb_node/feature_ros_tools.h"

namespace voxblox {
namespace voxblox_gsm {

std_msgs::ColorRGBA getColorFromBlockFeatures(
    const double max_number_of_features, const double number_of_features) {
  const double ratio =
      2 * std::min(1.0, number_of_features / max_number_of_features);
  const double r = std::max(0.0, ratio - 1.0);
  const double b = std::max(0.0, 1.0 - ratio);

  std_msgs::ColorRGBA color_msg;
  color_msg.r = r;
  color_msg.g = 1.0 - b - r;
  color_msg.b = b;
  color_msg.a = 0.2;

  return color_msg;
}

void fromFeaturesMsgToFeature3D(const modelify_msgs::Features& features_msg,
                                size_t* descriptor_size,
                                std::string* camera_frame, ros::Time* timestamp,
                                std::vector<Feature3D>* features_C) {
  CHECK_NOTNULL(camera_frame);
  CHECK_NOTNULL(timestamp);
  CHECK_NOTNULL(features_C);

  *descriptor_size = features_msg.descriptor_length;
  *camera_frame = features_msg.header.frame_id;
  *timestamp = features_msg.header.stamp;

  for (const modelify_msgs::Feature& msg : features_msg.features) {
    Feature3D feature;

    feature.keypoint << msg.x, msg.y, msg.z;
    feature.keypoint_scale = msg.scale;
    feature.keypoint_response = msg.response;
    feature.keypoint_angle = msg.angle;

    constexpr size_t kRows = 1;
    feature.descriptor = cv::Mat(kRows, *descriptor_size, CV_32FC1);
    memcpy(feature.descriptor.data, msg.descriptor.data(),
           msg.descriptor.size() * sizeof(float));

    CHECK_EQ(feature.descriptor.cols, msg.descriptor.size())
        << "Descriptor size is wrong!";

    features_C->push_back(feature);
  }
}

}  // namespace voxblox_gsm
}  // namespace voxblox
