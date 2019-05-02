#ifndef GLOBAL_FEATURE_MAP_FEATURE_UTILS_INL_H_
#define GLOBAL_FEATURE_MAP_FEATURE_UTILS_INL_H_

#include <memory>
#include <string>
#include <utility>

#include <glog/logging.h>
#include <voxblox/core/common.h>

#include "global_feature_map/feature_utils.h"

namespace voxblox {

template <typename FeatureType>
void centerBlocksOfFeatureLayer(FeatureLayer<FeatureType>* layer,
                                Point* new_layer_origin) {
  CHECK_NOTNULL(layer);

  // Compute the exact cenroid of all allocated block indices.
  Point centroid = Point::Zero();
  BlockIndexList block_indices;
  layer->getAllAllocatedBlocks(&block_indices);
  for (const BlockIndex block_index : block_indices) {
    centroid += layer->getBlockByIndex(block_index).origin();
  }
  centroid /= static_cast<FloatingPoint>(block_indices.size());

  // Round to nearest block index to centroid.
  centroid /= layer->block_size();
  const BlockIndex index_centroid =
      (centroid + 0.5 * Point::Ones()).cast<IndexElement>();

  // Return the new origin expressed in the old origins coordinate frame.
  const FloatingPoint block_size = layer->block_size();
  *new_layer_origin = index_centroid.cast<FloatingPoint>() * block_size;

  VLOG(3) << "The new origin of the coordinate frame (expressed in the old "
          << "coordinate frame) is: " << new_layer_origin->transpose();

  // Loop over all blocks and change their spatial indices.
  // The only way to do this is to remove them all, store them in a temporary
  // hash map and re-insert them again. This sounds worse than it is, the blocks
  // are all shared ptrs and therefore only a negligible amount of real memory
  // operations is necessary.
  const size_t num_allocated_blocks_before =
      layer->getNumberOfAllocatedBlocks();
  typename FeatureLayer<FeatureType>::FeatureBlockHashMap temporary_map;
  for (const BlockIndex& block_index : block_indices) {
    typename FeatureBlock<FeatureType>::Ptr block_ptr =
        layer->getBlockPtrByIndex(block_index);
    const Point new_origin = block_ptr->origin() - *new_layer_origin;
    block_ptr->setOrigin(new_origin);
    block_ptr->shiftFeatures(*new_layer_origin);

    // Extract block and shift block index.
    temporary_map.emplace(block_index - index_centroid, block_ptr);
  }

  layer->removeAllBlocks();
  CHECK_EQ(layer->getNumberOfAllocatedBlocks(), 0u);

  // Insert into layer again.
  for (const std::pair<const BlockIndex,
                       typename FeatureBlock<FeatureType>::Ptr>&
           idx_block_pair : temporary_map) {
    layer->insertBlock(idx_block_pair);
  }
  CHECK_EQ(layer->getNumberOfAllocatedBlocks(), num_allocated_blocks_before);
}

template <typename FeatureType>
void mergeFeatureLayerAintoFeatureLayerB(
    const FeatureLayer<FeatureType>& layer_A, const Transformation& T_B_A,
    FeatureLayer<FeatureType>* layer_B) {
  FeatureLayer<FeatureType> layer_A_transformed(layer_B->block_size());

  transformFeatureLayer(layer_A, T_B_A, &layer_A_transformed);

  mergeFeatureLayerAintoFeatureLayerB(layer_A_transformed, layer_B);
}

template <typename FeatureType>
void mergeFeatureLayerAintoFeatureLayerB(
    const FeatureLayer<FeatureType>& layer_A,
    FeatureLayer<FeatureType>* layer_B) {
  CHECK_NOTNULL(layer_B);
  const FeatureLayer<FeatureType>* layer_A_ptr = &layer_A;

  BlockIndexList block_idx_list_A;
  layer_A.getAllAllocatedBlocks(&block_idx_list_A);

  for (const BlockIndex& block_idx : block_idx_list_A) {
    typename FeatureBlock<FeatureType>::ConstPtr block_A_ptr =
        layer_A_ptr->getBlockPtrByIndex(block_idx);
    typename FeatureBlock<FeatureType>::Ptr block_B_ptr =
        layer_B->getBlockPtrByIndex(block_idx);

    if (!block_B_ptr) {
      block_B_ptr = layer_B->allocateBlockPtrByIndex(block_idx);
    }

    if ((block_A_ptr != nullptr) && (block_B_ptr != nullptr)) {
      block_B_ptr->mergeBlock(*block_A_ptr);
    }
  }
}

template <typename FeatureType>
void transformFeatureLayer(const FeatureLayer<FeatureType>& layer_in,
                           const Transformation& T_out_in,
                           FeatureLayer<FeatureType>* layer_out) {
  CHECK_NOTNULL(layer_out);

  // TODO(ntonci): Make config a param.
  FeatureIntegrator::Config feature_integrator_config;
  FeatureIntegrator feature_integrator(feature_integrator_config, layer_out);

  std::vector<FeatureType> features;
  layer_in.getFeatures(&features);

  feature_integrator.integrateFeatures(T_out_in, features);
}

}  // namespace voxblox

#endif  // GLOBAL_FEATURE_MAP_FEATURE_UTILS_INL_H_
