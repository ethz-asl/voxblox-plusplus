#include "global_feature_map/feature_integrator.h"

namespace voxblox {

void FeatureIntegrator::integrateFeatures(
    const Transformation& T_G_C, const std::vector<Feature3D>& features) {
  timing::Timer integrate_timer("integrate/features");

  ThreadSafeIndex index_getter(features.size());

  std::list<std::thread> integration_threads;
  for (size_t i = 0; i < config_.integrator_threads; ++i) {
    integration_threads.emplace_back(&FeatureIntegrator::integrateFunction,
                                     this, T_G_C, features, &index_getter);
  }

  for (std::thread& thread : integration_threads) {
    thread.join();
  }
  integrate_timer.Stop();

  timing::Timer insertion_timer("inserting_missed_blocks");
  updateLayerWithStoredBlocks();
  insertion_timer.Stop();
}

void FeatureIntegrator::integrateFunction(
    const Transformation& T_G_C, const std::vector<Feature3D>& features,
    ThreadSafeIndex* index_getter) {
  DCHECK(index_getter != nullptr);

  size_t point_idx;
  while (index_getter->getNextIndex(&point_idx)) {
    const Feature3D& feature = features.at(point_idx);

    Point point_C;
    point_C << feature.keypoint.x, feature.keypoint.y, feature.keypoint.z;
    const Point origin = T_G_C.getPosition();
    const Point point_G = T_G_C * point_C;

    FeatureBlock<Feature3D>::Ptr block = nullptr;
    BlockIndex block_idx;

    allocateStorageAndGetBlockPtr(point_G, &block, &block_idx);
    updateFeatureBlock(feature, &block);
  }
}

void FeatureIntegrator::allocateStorageAndGetBlockPtr(
    const Point& point_G, FeatureBlock<Feature3D>::Ptr* last_block,
    BlockIndex* last_block_idx) {
  DCHECK(last_block != nullptr);
  DCHECK(last_block_idx != nullptr);

  const BlockIndex block_idx =
      layer_->computeBlockIndexFromCoordinates(point_G);

  if ((block_idx != *last_block_idx) || (*last_block == nullptr)) {
    *last_block = layer_->getBlockPtrByIndex(block_idx);
    *last_block_idx = block_idx;
  }

  if (*last_block == nullptr) {
    std::lock_guard<std::mutex> lock(temp_block_mutex_);

    FeatureLayer<Feature3D>::FeatureBlockHashMap::iterator it =
        temp_block_map_.find(block_idx);
    if (it != temp_block_map_.end()) {
      *last_block = it->second;
    } else {
      auto insert_status = temp_block_map_.emplace(
          block_idx, std::make_shared<FeatureBlock<Feature3D>>(
                         block_size_,
                         getOriginPointFromGridIndex(block_idx, block_size_)));

      DCHECK(insert_status.second) << "Block already exists when allocating at "
                                   << block_idx.transpose();

      *last_block = insert_status.first->second;
    }
  }

  (*last_block)->updated() = true;
}

void FeatureIntegrator::updateFeatureBlock(
    const Feature3D& feature, FeatureBlock<Feature3D>::Ptr* block) {
  DCHECK(block != nullptr);

  std::lock_guard<std::mutex> lock((*block)->getMutex());

  (*block)->addFeature(feature);
  (*block)->set_has_data(true);
}

void FeatureIntegrator::updateLayerWithStoredBlocks() {
  BlockIndex last_block_idx;
  FeatureBlock<Feature3D>::Ptr block = nullptr;

  for (const std::pair<const BlockIndex, FeatureBlock<Feature3D>::Ptr>&
           temp_block_pair : temp_block_map_) {
    const BlockIndex block_idx = temp_block_pair.first;
    FeatureBlock<Feature3D>::Ptr block = temp_block_pair.second;
    std::vector<Feature3D> features = block->getFeatures();

    FeatureBlock<Feature3D>::Ptr original_block =
        layer_->allocateBlockPtrByIndex(block_idx);

    // TODO(ntonci): Do the sorting here or in modelify.
    for (Feature3D& feature : features) {
      original_block->addFeature(feature);
    }
  }

  temp_block_map_.clear();
}

}  // namespace voxblox
