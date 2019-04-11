#include <gtest/gtest.h>

#include "global_feature_map/feature_block.h"
#include "global_feature_map/feature_layer.h"
#include "global_feature_map/feature_types.h"

using namespace voxblox;  // NOLINT

template <typename FeatureType>
class FeatureLayerTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    feature_layer_.reset(new FeatureLayer<FeatureType>(kBlockSize));
  }

  typename FeatureLayer<FeatureType>::Ptr feature_layer_;

  const FloatingPoint kBlockSize =
      0.16;  // corresponds to 16cm = 8x8x8 @ 2cm voxel resolution
};

typedef FeatureLayerTest<Feature3D> Feature3DLayerTest;

TEST_F(Feature3DLayerTest, DoStuff) {
  BlockIndex block_index(-15, 20, 3);

  FeatureBlock<Feature3D>::Ptr test_block =
      feature_layer_->allocateBlockPtrByIndex(block_index);
  CHECK(test_block);

  Feature3D test_feature;
  test_block->addFeature(test_feature);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  int result = RUN_ALL_TESTS();

  return result;
}
