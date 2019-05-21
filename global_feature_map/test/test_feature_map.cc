#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include "global_feature_map/feature_block.h"
#include "global_feature_map/feature_integrator.h"
#include "global_feature_map/feature_layer.h"
#include "global_feature_map/feature_types.h"
#include "global_feature_map/feature_utils.h"

using namespace voxblox;  // NOLINT

template <typename FeatureType>
class FeatureLayerTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    feature_layer_.reset(new FeatureLayer<FeatureType>(kBlockSize));
    feature_layer_deserialize_.reset(new FeatureLayer<FeatureType>(kBlockSize));
    second_feature_layer_.reset(new FeatureLayer<FeatureType>(kBlockSize));

    integrator_config_.match_descriptors_to_reject_duplicates = false;
    feature_integrator_.reset(
        new FeatureIntegrator(integrator_config_, feature_layer_.get()));
    second_feature_integrator_.reset(
        new FeatureIntegrator(integrator_config_, second_feature_layer_.get()));
  }

  typename FeatureLayer<FeatureType>::Ptr feature_layer_;
  typename FeatureLayer<FeatureType>::Ptr feature_layer_deserialize_;
  typename FeatureLayer<FeatureType>::Ptr second_feature_layer_;
  FeatureIntegrator::Config integrator_config_;
  FeatureIntegrator::Ptr feature_integrator_;
  FeatureIntegrator::Ptr second_feature_integrator_;

  const FloatingPoint kBlockSize =
      0.1;  // corresponds to 16cm = 8x8x8 @ 2cm voxel resolution
  const Point kOrigin = Point(0.0, 0.0, 0.0);
  const FloatingPoint kErrorThreshold = 1e-6;
};

typedef FeatureLayerTest<Feature3D> Feature3DLayerTest;

TEST_F(Feature3DLayerTest, testBlockFunctionality) {
  BlockIndex block_index(-5, 5, 5);

  FeatureBlock<Feature3D>::Ptr test_block =
      feature_layer_->allocateBlockPtrByIndex(block_index);
  CHECK(test_block);

  Point block_origin(-0.5, 0.5, 0.5);
  EXPECT_EQ(test_block->origin(), block_origin);

  Feature3D test_feature;
  test_feature.keypoint = Point(-0.6, 0.6, 0.6);
  test_block->addFeature(test_feature);
  EXPECT_EQ(test_block->numFeatures(), 1u);
  test_feature.keypoint = Point(-0.7, 0.7, 0.7);
  test_block->addFeature(test_feature);
  EXPECT_EQ(test_block->numFeatures(), 2u);
  test_feature.keypoint = Point(-0.4, 0.4, 0.4);
  test_block->addFeature(test_feature);
  EXPECT_EQ(test_block->numFeatures(), 3u);

  test_block->shiftBlockWithFeatures(block_origin);
  EXPECT_NEAR(
      (test_block->getFeature(0u).keypoint - Point(-0.1, 0.1, 0.1)).sum(), 0.0,
      kErrorThreshold);
  EXPECT_NEAR(
      (test_block->getFeature(1u).keypoint - Point(-0.2, 0.2, 0.2)).sum(), 0.0,
      kErrorThreshold);
  EXPECT_NEAR(
      (test_block->getFeature(2u).keypoint - Point(0.1, -0.1, -0.1)).sum(), 0.0,
      kErrorThreshold);

  FeatureBlock<Feature3D>::Ptr second_test_block(
      new FeatureBlock<Feature3D>(kBlockSize, block_origin));
  second_test_block->addFeature(test_feature);

  test_block->mergeBlock(*second_test_block);
  EXPECT_NEAR(
      (test_block->getFeature(3u).keypoint - test_feature.keypoint).sum(), 0.0,
      kErrorThreshold);
}

TEST_F(Feature3DLayerTest, testLayerFunctionality) {
  BlockIndex block_index =
      feature_layer_->computeBlockIndexFromCoordinates(kOrigin);
  EXPECT_EQ(block_index, BlockIndex(0, 0, 0));

  FeatureBlock<Feature3D>::Ptr test_block(
      new FeatureBlock<Feature3D>(kBlockSize, kOrigin));
  Feature3D test_feature;
  test_feature.keypoint = Point(-0.6, 0.6, 0.6);
  test_block->addFeature(test_feature);
  test_feature.keypoint = Point(-0.7, 0.7, 0.7);
  test_block->addFeature(test_feature);

  feature_layer_->insertBlock(std::make_pair(block_index, test_block));

  BlockIndexList all_blocks;
  feature_layer_->getAllAllocatedBlocks(&all_blocks);
  EXPECT_EQ(all_blocks.size(), 1u);

  BlockIndex second_block_index(1, 1, 1);
  FeatureBlock<Feature3D>::Ptr second_test_block(new FeatureBlock<Feature3D>(
      kBlockSize, Point(kBlockSize, kBlockSize, kBlockSize)));
  test_feature.keypoint = Point(-0.1, 0.1, 0.1);
  second_test_block->addFeature(test_feature);
  test_feature.keypoint = Point(-0.2, 0.2, 0.2);
  second_test_block->addFeature(test_feature);

  feature_layer_->insertBlock(
      std::make_pair(second_block_index, second_test_block));
  feature_layer_->getAllAllocatedBlocks(&all_blocks);
  EXPECT_EQ(all_blocks.size(), 2u);

  std::vector<Feature3D> features;
  feature_layer_->getFeatures(&features);
  EXPECT_EQ(features.size(), 4u);
  EXPECT_NEAR((features.at(0).keypoint - Point(-0.1, 0.1, 0.1)).sum(), 0.0,
              kErrorThreshold);
  EXPECT_NEAR((features.at(1).keypoint - Point(-0.2, 0.2, 0.2)).sum(), 0.0,
              kErrorThreshold);
  EXPECT_NEAR((features.at(2).keypoint - Point(-0.6, 0.6, 0.6)).sum(), 0.0,
              kErrorThreshold);
  EXPECT_NEAR((features.at(3).keypoint - Point(-0.7, 0.7, 0.7)).sum(), 0.0,
              kErrorThreshold);

  feature_layer_->removeAllBlocks();
  feature_layer_->getAllAllocatedBlocks(&all_blocks);
  EXPECT_EQ(all_blocks.size(), 0u);

  feature_layer_->insertBlock(std::make_pair(block_index, test_block));
  feature_layer_->getAllAllocatedBlocks(&all_blocks);
  EXPECT_EQ(all_blocks.size(), 1u);
  feature_layer_->removeBlock(block_index);
  feature_layer_->getAllAllocatedBlocks(&all_blocks);
  EXPECT_EQ(all_blocks.size(), 0u);
}

TEST_F(Feature3DLayerTest, testSerialization) {
  BlockIndex block_index(-15, 20, 3);

  const size_t kDescriptorSize = 128u;

  FeatureBlock<Feature3D>::Ptr test_block =
      feature_layer_->allocateBlockPtrByIndex(block_index);
  CHECK(test_block);

  feature_layer_->setDescriptorSize(kDescriptorSize);

  Feature3D test_feature_a;
  test_feature_a.keypoint << 1.1, 1.1, 1.1;
  test_feature_a.keypoint_scale = 11.1;
  test_feature_a.keypoint_response = 111.1;
  test_feature_a.keypoint_angle = 1.1;
  test_feature_a.descriptor =
      cv::Mat(1, kDescriptorSize, CV_32FC1, cv::Scalar(1.11));
  test_block->addFeature(test_feature_a);
  Feature3D test_feature_b;
  test_feature_b.keypoint << 2.2, 2.2, 2.2;
  test_feature_b.keypoint_scale = 22.2;
  test_feature_b.keypoint_response = 222.2;
  test_feature_b.keypoint_angle = 2.2;
  test_feature_b.descriptor =
      cv::Mat(1, kDescriptorSize, CV_32FC1, cv::Scalar(2.22));
  test_block->addFeature(test_feature_b);

  constexpr bool kOnlyUpdated = false;
  modelify_msgs::FeatureLayer msg;
  feature_layer_->serializeLayerAsMsg(kOnlyUpdated, DeserializeAction::kUpdate,
                                      &msg);

  feature_layer_deserialize_->setDescriptorSize(kDescriptorSize);
  feature_layer_deserialize_->deserializeMsgToLayer(msg);

  FeatureBlock<Feature3D>::Ptr test_block_deserialize =
      feature_layer_deserialize_->getBlockPtrByIndex(block_index);
  std::vector<Feature3D>& features_deserialize =
      test_block_deserialize->getFeatures();
  Feature3D test_feature_a_deserialize = features_deserialize.at(0);
  Feature3D test_feature_b_deserialize = features_deserialize.at(1);

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

TEST_F(Feature3DLayerTest, testSerializationLargerDescriptor) {
  BlockIndex block_index(-15, 20, 3);

  const size_t kDescriptorSize = 256u;

  FeatureBlock<Feature3D>::Ptr test_block =
      feature_layer_->allocateBlockPtrByIndex(block_index);
  CHECK(test_block);

  feature_layer_->setDescriptorSize(kDescriptorSize);

  Feature3D test_feature_a;
  test_feature_a.keypoint << 1.1, 1.1, 1.1;
  test_feature_a.keypoint_scale = 11.1;
  test_feature_a.keypoint_response = 111.1;
  test_feature_a.keypoint_angle = 1.1;
  test_feature_a.descriptor =
      cv::Mat(1, kDescriptorSize, CV_32FC1, cv::Scalar(1.11));
  test_block->addFeature(test_feature_a);
  Feature3D test_feature_b;
  test_feature_b.keypoint << 2.2, 2.2, 2.2;
  test_feature_b.keypoint_scale = 22.2;
  test_feature_b.keypoint_response = 222.2;
  test_feature_b.keypoint_angle = 2.2;
  test_feature_b.descriptor =
      cv::Mat(1, kDescriptorSize, CV_32FC1, cv::Scalar(2.22));
  test_block->addFeature(test_feature_b);

  constexpr bool kOnlyUpdated = false;
  modelify_msgs::FeatureLayer msg;
  feature_layer_->serializeLayerAsMsg(kOnlyUpdated, DeserializeAction::kUpdate,
                                      &msg);

  feature_layer_deserialize_->setDescriptorSize(kDescriptorSize);
  feature_layer_deserialize_->deserializeMsgToLayer(msg);

  FeatureBlock<Feature3D>::Ptr test_block_deserialize =
      feature_layer_deserialize_->getBlockPtrByIndex(block_index);
  std::vector<Feature3D>& features_deserialize =
      test_block_deserialize->getFeatures();
  Feature3D test_feature_a_deserialize = features_deserialize.at(0);
  Feature3D test_feature_b_deserialize = features_deserialize.at(1);

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

TEST_F(Feature3DLayerTest, testIntegratorFunctionality) {
  Eigen::Matrix<FloatingPoint, 4, 4> T_C1;
  T_C1 << 1.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
      0.0, 1.0;
  Transformation T_G_C1(T_C1);
  Feature3D feature;
  std::vector<Feature3D> features_C1;
  feature.keypoint = Point(0.001, 0.051, 0.001);
  features_C1.push_back(feature);
  feature.keypoint = Point(0.051, 0.051, 0.151);
  features_C1.push_back(feature);
  feature.keypoint = Point(0.151, 0.251, 0.251);
  features_C1.push_back(feature);
  feature_integrator_->integrateFeatures(T_G_C1, features_C1);

  BlockIndexList all_blocks;
  feature_layer_->getAllAllocatedBlocks(&all_blocks);
  EXPECT_EQ(all_blocks.size(), 3u);

  Eigen::Matrix<FloatingPoint, 4, 4> T_C2;
  T_C2 << 1.0, 0.0, 0.0, 0.2, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
      0.0, 1.0;
  Transformation T_G_C2(T_C2);
  std::vector<Feature3D> features_C2;
  feature.keypoint = Point(-0.051, 0.051, 0.001);
  features_C2.push_back(feature);
  feature.keypoint = Point(0.001, 0.251, 0.001);
  features_C2.push_back(feature);
  feature.keypoint = Point(0.051, 0.251, 0.275);
  features_C2.push_back(feature);
  feature_integrator_->integrateFeatures(T_G_C2, features_C2);

  feature_layer_->getAllAllocatedBlocks(&all_blocks);
  EXPECT_EQ(all_blocks.size(), 4u);

  BlockIndex block_index1(2, 2, 0);
  BlockIndex block_index2(1, 0, 0);
  BlockIndex block_index3(2, 2, 2);
  BlockIndex block_index4(1, 0, 1);

  FeatureBlock<Feature3D>::Ptr block_1 =
      feature_layer_->allocateBlockPtrByIndex(block_index1);
  FeatureBlock<Feature3D>::Ptr block_2 =
      feature_layer_->allocateBlockPtrByIndex(block_index2);
  FeatureBlock<Feature3D>::Ptr block_3 =
      feature_layer_->allocateBlockPtrByIndex(block_index3);
  FeatureBlock<Feature3D>::Ptr block_4 =
      feature_layer_->allocateBlockPtrByIndex(block_index4);

  EXPECT_NEAR((block_1->origin() - Point(0.2, 0.2, 0.0)).sum(), 0.0,
              kErrorThreshold);
  EXPECT_EQ(block_1->numFeatures(), 1u);
  EXPECT_NEAR(
      (block_1->getFeature(0u).keypoint - Point(0.201, 0.251, 0.001)).sum(),
      0.0, kErrorThreshold);
  EXPECT_NEAR((block_2->origin() - Point(0.1, 0.0, 0.0)).sum(), 0.0,
              kErrorThreshold);
  EXPECT_EQ(block_2->numFeatures(), 2u);
  EXPECT_NEAR(
      (block_2->getFeature(0u).keypoint - Point(0.101, 0.051, 0.001)).sum(),
      0.0, kErrorThreshold);
  EXPECT_NEAR(
      (block_2->getFeature(1u).keypoint - Point(0.149, 0.051, 0.001)).sum(),
      0.0, kErrorThreshold);
  EXPECT_NEAR((block_3->origin() - Point(0.2, 0.2, 0.2)).sum(), 0.0,
              kErrorThreshold);
  EXPECT_EQ(block_3->numFeatures(), 2u);
  EXPECT_NEAR(
      (block_3->getFeature(0u).keypoint - Point(0.251, 0.251, 0.251)).sum(),
      0.0, kErrorThreshold);
  EXPECT_NEAR(
      (block_3->getFeature(1u).keypoint - Point(0.251, 0.251, 0.275)).sum(),
      0.0, kErrorThreshold);
  EXPECT_NEAR((block_4->origin() - Point(0.1, 0.0, 0.1)).sum(), 0.0,
              kErrorThreshold);
  EXPECT_EQ(block_4->numFeatures(), 1u);
  EXPECT_NEAR(
      (block_4->getFeature(0u).keypoint - Point(0.151, 0.051, 0.151)).sum(),
      0.0, kErrorThreshold);

  std::vector<Feature3D> all_features;
  feature_layer_->getFeatures(&all_features);
  EXPECT_EQ(all_features.size(), 6u);
}

TEST_F(Feature3DLayerTest, testUtilsFunctionality) {
  constexpr size_t kDescriptorSize = 128u;

  std::vector<Feature3D> features;
  Feature3D feature;
  feature.keypoint_scale = 1.0;
  feature.keypoint_response = 1.0;
  feature.keypoint_angle = 1.0;
  feature.keypoint = Point(0.255, 0.155, 0.001);
  feature.descriptor = cv::Mat(1, kDescriptorSize, CV_32FC1, cv::Scalar(1));
  features.push_back(feature);
  feature.keypoint = Point(0.455, 0.355, 0.001);
  feature.descriptor = cv::Mat(1, kDescriptorSize, CV_32FC1, cv::Scalar(2));
  features.push_back(feature);
  feature.keypoint = Point(0.475, 0.375, 0.001);
  feature.descriptor = cv::Mat(1, kDescriptorSize, CV_32FC1, cv::Scalar(3));
  features.push_back(feature);
  feature.keypoint = Point(0.655, 0.155, 0.001);
  feature.descriptor = cv::Mat(1, kDescriptorSize, CV_32FC1, cv::Scalar(4));
  features.push_back(feature);
  feature.keypoint = Point(0.675, 0.175, 0.001);
  feature.descriptor = cv::Mat(1, kDescriptorSize, CV_32FC1, cv::Scalar(5));
  features.push_back(feature);
  feature.keypoint = Point(0.625, 0.125, 0.001);
  feature.descriptor = cv::Mat(1, kDescriptorSize, CV_32FC1, cv::Scalar(6));
  features.push_back(feature);

  Eigen::Matrix<FloatingPoint, 4, 4> T_C0;
  T_C0 << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
      0.0, 1.0;
  Transformation T_G_C0(T_C0);
  feature_integrator_->integrateFeatures(T_G_C0, features);

  CHECK_EQ(feature_layer_->getDescriptorSize(), kDescriptorSize);

  Point new_layer_origin;
  centerBlocksOfFeatureLayer(feature_layer_.get(), &new_layer_origin);

  BlockIndex block_index1(-2, -1, 0);
  BlockIndex block_index2(0, 1, 0);
  BlockIndex block_index3(2, -1, 0);

  FeatureBlock<Feature3D>::Ptr block_1 =
      feature_layer_->getBlockPtrByIndex(block_index1);
  FeatureBlock<Feature3D>::Ptr block_2 =
      feature_layer_->getBlockPtrByIndex(block_index2);
  FeatureBlock<Feature3D>::Ptr block_3 =
      feature_layer_->getBlockPtrByIndex(block_index3);

  BlockIndexList all_blocks;
  feature_layer_->getAllAllocatedBlocks(&all_blocks);
  EXPECT_EQ(all_blocks.size(), 3u);

  EXPECT_NE(block_1.get(), nullptr);
  EXPECT_NE(block_2.get(), nullptr);
  EXPECT_NE(block_3.get(), nullptr);
  EXPECT_NEAR((new_layer_origin - Point(0.4, 0.2, 0.0)).sum(), 0.0,
              kErrorThreshold);

  EXPECT_NEAR(
      (block_1->getFeature(0u).keypoint - Point(-0.145, -0.045, 0.001)).sum(),
      0.0, kErrorThreshold);
  EXPECT_NEAR((block_2->getFeature(0u).keypoint +
               block_2->getFeature(1u).keypoint - Point(0.130, 0.330, 0.002))
                  .sum(),
              0.0, kErrorThreshold);
  EXPECT_NEAR(
      (block_3->getFeature(0u).keypoint + block_3->getFeature(1u).keypoint +
       block_3->getFeature(2u).keypoint - Point(0.755, -0.145, 0.003))
          .sum(),
      0.0, kErrorThreshold);

  Eigen::Matrix<FloatingPoint, 4, 4> T_C1;
  T_C1 << 0.0, -1.0, 0.0, 0.05, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 1.0;
  Transformation T_G_C1(T_C1);

  second_feature_integrator_->integrateFeatures(T_G_C0, features);

  second_feature_layer_->setDescriptorSize(kDescriptorSize);

  mergeFeatureLayerAintoFeatureLayerB(*feature_layer_, T_G_C1,
                                      second_feature_layer_.get());

  second_feature_layer_->getAllAllocatedBlocks(&all_blocks);
  EXPECT_EQ(all_blocks.size(), 7u);

  BlockIndex second_block_index1(0, 2, 0);
  BlockIndex second_block_index2(-2, 0, 0);
  BlockIndex second_block_index3(1, 2, 0);
  BlockIndex second_block_index4(0, -2, 0);
  BlockIndex second_block_index5(2, 1, 0);
  BlockIndex second_block_index6(4, 3, 0);
  BlockIndex second_block_index7(6, 1, 0);

  FeatureBlock<Feature3D>::Ptr second_block_1 =
      second_feature_layer_->getBlockPtrByIndex(second_block_index1);
  FeatureBlock<Feature3D>::Ptr second_block_2 =
      second_feature_layer_->getBlockPtrByIndex(second_block_index2);
  FeatureBlock<Feature3D>::Ptr second_block_3 =
      second_feature_layer_->getBlockPtrByIndex(second_block_index3);
  FeatureBlock<Feature3D>::Ptr second_block_4 =
      second_feature_layer_->getBlockPtrByIndex(second_block_index4);
  FeatureBlock<Feature3D>::Ptr second_block_5 =
      second_feature_layer_->getBlockPtrByIndex(second_block_index5);
  FeatureBlock<Feature3D>::Ptr second_block_6 =
      second_feature_layer_->getBlockPtrByIndex(second_block_index6);
  FeatureBlock<Feature3D>::Ptr second_block_7 =
      second_feature_layer_->getBlockPtrByIndex(second_block_index7);

  EXPECT_NE(second_block_1.get(), nullptr);
  EXPECT_NE(second_block_2.get(), nullptr);
  EXPECT_NE(second_block_3.get(), nullptr);
  EXPECT_NE(second_block_4.get(), nullptr);
  EXPECT_NE(second_block_5.get(), nullptr);
  EXPECT_NE(second_block_6.get(), nullptr);
  EXPECT_NE(second_block_7.get(), nullptr);

  EXPECT_NEAR(
      (second_block_1->getFeature(0u).keypoint +
       second_block_1->getFeature(1u).keypoint - Point(0.170, 0.530, 0.002))
          .sum(),
      0.0, kErrorThreshold);
  EXPECT_NEAR(
      (second_block_2->getFeature(0u).keypoint +
       second_block_2->getFeature(1u).keypoint - Point(-0.230, 0.130, 0.002))
          .sum(),
      0.0, kErrorThreshold);
  EXPECT_NEAR(
      (second_block_3->getFeature(0u).keypoint - Point(0.125, 0.225, 0.001))
          .sum(),
      0.0, kErrorThreshold);
  EXPECT_NEAR(
      (second_block_4->getFeature(0u).keypoint - Point(0.095, -0.145, 0.001))
          .sum(),
      0.0, kErrorThreshold);
  EXPECT_NEAR(
      (second_block_5->getFeature(0u).keypoint - Point(0.255, 0.155, 0.001))
          .sum(),
      0.0, kErrorThreshold);
  EXPECT_NEAR(
      (second_block_6->getFeature(0u).keypoint +
       second_block_6->getFeature(1u).keypoint - Point(0.930, 0.730, 0.002))
          .sum(),
      0.0, kErrorThreshold);
  EXPECT_NEAR(
      (second_block_7->getFeature(0u).keypoint +
       second_block_7->getFeature(1u).keypoint +
       second_block_7->getFeature(2u).keypoint - Point(1.955, 0.455, 0.003))
          .sum(),
      0.0, kErrorThreshold);

  bool success_save = saveFeatureLayer<Feature3D>(*second_feature_layer_,
                                                  "/tmp/feature_layer.proto");

  FeatureLayer<Feature3D>::Ptr load_feature_layer(
      new FeatureLayer<Feature3D>(kBlockSize, kDescriptorSize));
  bool success_load = loadFeatureLayer<Feature3D>("/tmp/feature_layer.proto",
                                                  &load_feature_layer);

  EXPECT_EQ(second_feature_layer_->getNumberOfAllocatedBlocks(),
            load_feature_layer->getNumberOfAllocatedBlocks());
  EXPECT_EQ(second_feature_layer_->block_size(),
            load_feature_layer->block_size());
  EXPECT_EQ(second_feature_layer_->getDescriptorSize(),
            load_feature_layer->getDescriptorSize());
  EXPECT_EQ(second_feature_layer_->getMemorySize(),
            load_feature_layer->getMemorySize());

  second_feature_layer_->getAllAllocatedBlocks(&all_blocks);

  for (BlockIndex& block : all_blocks) {
    FeatureBlock<Feature3D>::Ptr block_first =
        second_feature_layer_->getBlockPtrByIndex(block);
    FeatureBlock<Feature3D>::Ptr block_second =
        load_feature_layer->getBlockPtrByIndex(block);

    EXPECT_NE(block_first.get(), nullptr);
    EXPECT_NE(block_second.get(), nullptr);

    EXPECT_EQ(block_first->numFeatures(), block_second->numFeatures());
    EXPECT_EQ(block_first->getMemorySize(), block_second->getMemorySize());
    EXPECT_EQ(block_first->block_size(), block_second->block_size());

    EXPECT_NEAR((block_first->origin() - block_second->origin()).sum(), 0.0,
                kErrorThreshold);
    EXPECT_NEAR(
        (block_first->block_index() - block_second->block_index()).sum(), 0.0,
        kErrorThreshold);

    std::vector<Feature3D> features_first = block_first->getFeatures();
    std::vector<Feature3D> features_second = block_second->getFeatures();

    EXPECT_EQ(features_first.size(), features_second.size());

    for (size_t i = 0u; i < features_first.size(); ++i) {
      Feature3D feature_first = features_first.at(i);
      Feature3D feature_second = features_second.at(i);

      EXPECT_EQ(feature_first.keypoint, feature_second.keypoint);
      EXPECT_EQ(feature_first.keypoint_scale, feature_second.keypoint_scale);
      EXPECT_EQ(feature_first.keypoint_response,
                feature_second.keypoint_response);
      EXPECT_EQ(feature_first.keypoint_angle, feature_second.keypoint_angle);
      EXPECT_LT(
          cv::sum(feature_first.descriptor - feature_second.descriptor)[0],
          kErrorThreshold);
    }
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  int result = RUN_ALL_TESTS();

  return result;
}
