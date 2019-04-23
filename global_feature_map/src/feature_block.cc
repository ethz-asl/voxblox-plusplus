#include <opencv2/opencv.hpp>

#include "global_feature_map/feature_block.h"
#include "global_feature_map/feature_types.h"

namespace voxblox {

template <>
void FeatureBlock<Feature3D>::serializeToIntegers(
    std::vector<uint32_t>* data) const {
  CHECK_NOTNULL(data);

  //          Type            |  Bits
  // --------------------------------------
  // FloatingPoint x              4
  // FloatingPoint y              4
  // FloatingPoint z              4
  // FloatingPoint scale          4
  // FloatingPoint response       4
  // FloatingPoint angle          4
  // double descriptor            size*8, 1024 (for SIFT)

  const size_t number_of_features = numFeatures();

  constexpr size_t kFeatureBlockSizeWithoutDescriptor = 6u;
  // TODO(ntonci): Template on descirptor type.
  constexpr size_t kDescriptorSize = 128u;
  data->clear();
  data->reserve(number_of_features *
                (kFeatureBlockSizeWithoutDescriptor + kDescriptorSize));

  for (const Feature3D& feature : getFeatures()) {
    // TODO(ntonci): Currently Feature3D uses FloatingPoint type, however, if
    // this ever changes to double, conversion to uint32 will be wrong!
    const uint32_t* bytes_1_ptr =
        reinterpret_cast<const uint32_t*>(&feature.keypoint(0));
    data->push_back(*bytes_1_ptr);
    const uint32_t* bytes_2_ptr =
        reinterpret_cast<const uint32_t*>(&feature.keypoint(1));
    data->push_back(*bytes_2_ptr);
    const uint32_t* bytes_3_ptr =
        reinterpret_cast<const uint32_t*>(&feature.keypoint(2));
    data->push_back(*bytes_3_ptr);
    const uint32_t* bytes_4_ptr =
        reinterpret_cast<const uint32_t*>(&feature.keypoint_scale);
    data->push_back(*bytes_4_ptr);
    const uint32_t* bytes_5_ptr =
        reinterpret_cast<const uint32_t*>(&feature.keypoint_response);
    data->push_back(*bytes_5_ptr);
    const uint32_t* bytes_6_ptr =
        reinterpret_cast<const uint32_t*>(&feature.keypoint_angle);
    data->push_back(*bytes_6_ptr);

    // TODO(ntonci): Template on descriptor type.
    for (size_t i = 0u; i < kDescriptorSize; ++i) {
      // TODO(ntonci): Casting double to float, loosing precision. Check
      // significance.
      FloatingPoint descriptor_value =
          static_cast<FloatingPoint>(feature.descriptor.at<double>(i));
      const uint32_t* bytes_d_ptr =
          reinterpret_cast<const uint32_t*>(&descriptor_value);
      data->push_back(*bytes_d_ptr);
    }
  }
  CHECK_EQ(data->size(),
           number_of_features *
               (kFeatureBlockSizeWithoutDescriptor + kDescriptorSize));
}

template <>
void FeatureBlock<Feature3D>::deserializeFromIntegers(
    const std::vector<uint32_t>& data) {
  CHECK_EQ(numFeatures(), 0u);

  // TODO(ntonci): Template on descirptor type.
  constexpr size_t kFeature3DSize = 134u;
  constexpr size_t kDescriptorSize = 128u;

  const size_t num_data_packets = data.size();
  CHECK_EQ(num_data_packets % kFeature3DSize, 0);
  const size_t num_features = num_data_packets / kFeature3DSize;

  for (size_t data_idx = 0u; data_idx < num_data_packets;
       data_idx += kFeature3DSize) {
    const uint32_t bytes_1 = data[data_idx];
    const uint32_t bytes_2 = data[data_idx + 1u];
    const uint32_t bytes_3 = data[data_idx + 2u];
    const uint32_t bytes_4 = data[data_idx + 3u];
    const uint32_t bytes_5 = data[data_idx + 4u];
    const uint32_t bytes_6 = data[data_idx + 5u];

    Feature3D feature;

    memcpy(&(feature.keypoint(0)), &bytes_1, sizeof(bytes_1));
    memcpy(&(feature.keypoint(1)), &bytes_2, sizeof(bytes_2));
    memcpy(&(feature.keypoint(2)), &bytes_3, sizeof(bytes_3));
    memcpy(&feature.keypoint_scale, &bytes_4, sizeof(bytes_4));
    memcpy(&feature.keypoint_response, &bytes_5, sizeof(bytes_5));
    memcpy(&feature.keypoint_angle, &bytes_6, sizeof(bytes_6));

    feature.descriptor = cv::Mat(1u, kDescriptorSize, CV_64FC1);

    for (size_t j = 0u; j < kDescriptorSize; ++j) {
      float descriptor;
      const uint32_t bytes_d = data[data_idx + 6u + j];

      memcpy(&(descriptor), &bytes_d, sizeof(bytes_d));

      feature.descriptor.at<double>(j) = descriptor;
    }

    addFeature(feature);
  }
}

}  // namespace voxblox
