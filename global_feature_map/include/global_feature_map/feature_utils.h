#ifndef GLOBAL_FEATURE_MAP_FEATURE_UTILS_H_
#define GLOBAL_FEATURE_MAP_FEATURE_UTILS_H_

#include "global_feature_map/feature_block.h"
#include "global_feature_map/feature_integrator.h"
#include "global_feature_map/feature_layer.h"
#include "global_feature_map/feature_types.h"

namespace voxblox {

template <typename FeatureType>
void centerBlocksOfFeatureLayer(FeatureLayer<FeatureType>* layer,
                                Point* new_layer_origin);

template <typename FeatureType>
void mergeFeatureLayerAintoFeatureLayerB(
    const FeatureLayer<FeatureType>& layer_A, const Transformation& T_B_A,
    FeatureLayer<FeatureType>* layer_B);

template <typename FeatureType>
void mergeFeatureLayerAintoFeatureLayerB(
    const FeatureLayer<FeatureType>& layer_A,
    FeatureLayer<FeatureType>* layer_B);

template <typename FeatureType>
void transformFeatureLayer(const FeatureLayer<FeatureType>& layer_in,
                           const Transformation& T_out_in,
                           FeatureLayer<FeatureType>* layer_out);

}  // namespace voxblox

#endif  // GLOBAL_FEATURE_MAP_FEATURE_UTILS_H_

#include "global_feature_map/feature_utils_inl.h"
