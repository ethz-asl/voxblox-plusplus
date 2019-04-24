#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include "global_feature_map/feature_block.h"
#include "global_feature_map/feature_layer.h"
#include "global_feature_map/feature_types.h"

using namespace voxblox;  // NOLINT

template <typename FeatureType>
class FeatureLayerTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    feature_layer_.reset(new FeatureLayer<FeatureType>(kBlockSize));
    feature_layer_deserialize_.reset(new FeatureLayer<FeatureType>(kBlockSize));
  }

  typename FeatureLayer<FeatureType>::Ptr feature_layer_;
  typename FeatureLayer<FeatureType>::Ptr feature_layer_deserialize_;

  const FloatingPoint kBlockSize =
      0.16;  // corresponds to 16cm = 8x8x8 @ 2cm voxel resolution
};

typedef FeatureLayerTest<Feature3D> Feature3DLayerTest;

TEST_F(Feature3DLayerTest, testFeatureInsert) {
  BlockIndex block_index(-15, 20, 3);

  FeatureBlock<Feature3D>::Ptr test_block =
      feature_layer_->allocateBlockPtrByIndex(block_index);
  CHECK(test_block);

  Feature3D test_feature;
  test_block->addFeature(test_feature);
}

TEST_F(Feature3DLayerTest, testSerialization) {
  BlockIndex block_index(-15, 20, 3);

  FeatureBlock<Feature3D>::Ptr test_block =
      feature_layer_->allocateBlockPtrByIndex(block_index);
  CHECK(test_block);

  Feature3D test_feature_a;
  test_feature_a.keypoint << 1.1, 1.1, 1.1;
  test_feature_a.keypoint_scale = 11.1;
  test_feature_a.keypoint_response = 111.1;
  test_feature_a.keypoint_angle = 1.1;
  test_feature_a.descriptor = cv::Mat(1, 128, CV_64FC1, cv::Scalar(1.11));
  test_block->addFeature(test_feature_a);
  Feature3D test_feature_b;
  test_feature_b.keypoint << 2.2, 2.2, 2.2;
  test_feature_b.keypoint_scale = 22.2;
  test_feature_b.keypoint_response = 222.2;
  test_feature_b.keypoint_angle = 2.2;
  test_feature_b.descriptor = cv::Mat(1, 128, CV_64FC1, cv::Scalar(2.22));
  test_block->addFeature(test_feature_b);

  constexpr bool kOnlyUpdated = false;
  modelify_msgs::FeatureLayer msg;
  constexpr size_t action = 1u;
  feature_layer_->serializeLayerAsMsg(kOnlyUpdated, &msg, action);

  feature_layer_deserialize_->deserializeMsgToLayer(msg);

  FeatureBlock<Feature3D>::Ptr test_block_deserialize =
      feature_layer_deserialize_->getBlockPtrByIndex(block_index);
  std::vector<Feature3D>& features_deserialize =
      test_block_deserialize->getFeatures();
  Feature3D test_feature_a_deserialize = features_deserialize.at(0);
  Feature3D test_feature_b_deserialize = features_deserialize.at(1);

  constexpr double kErrorThreshold = 1e-5;
  EXPECT_EQ(feature_layer_->block_size(),
            feature_layer_deserialize_->block_size());
  EXPECT_EQ(feature_layer_->getNumberOfAllocatedBlocks(),
            feature_layer_deserialize_->getNumberOfAllocatedBlocks());
  EXPECT_EQ(test_block->numFeatures(), test_block_deserialize->numFeatures());
  EXPECT_EQ(test_feature_a.keypoint, test_feature_a_deserialize.keypoint);
  EXPECT_EQ(test_feature_a.keypoint_scale,
            test_feature_a_deserialize.keypoint_scale);
  EXPECT_EQ(test_feature_a.keypoint_response,
            test_feature_a_deserialize.keypoint_response);
  EXPECT_EQ(test_feature_a.keypoint_angle,
            test_feature_a_deserialize.keypoint_angle);
  EXPECT_LT(cv::sum(test_feature_a.descriptor -
                    test_feature_a_deserialize.descriptor)[0],
            kErrorThreshold);
  EXPECT_EQ(test_feature_b.keypoint, test_feature_b_deserialize.keypoint);
  EXPECT_EQ(test_feature_b.keypoint_scale,
            test_feature_b_deserialize.keypoint_scale);
  EXPECT_EQ(test_feature_b.keypoint_response,
            test_feature_b_deserialize.keypoint_response);
  EXPECT_EQ(test_feature_b.keypoint_angle,
            test_feature_b_deserialize.keypoint_angle);
  EXPECT_LT(cv::sum(test_feature_b.descriptor -
                    test_feature_b_deserialize.descriptor)[0],
            kErrorThreshold);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  int result = RUN_ALL_TESTS();

  return result;
}
