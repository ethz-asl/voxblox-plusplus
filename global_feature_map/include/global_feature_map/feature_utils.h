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

template <typename FeatureType>
bool loadFeatureBlocksFromFile(const std::string& file_path,
                               FeatureBlockMergingStrategy strategy,
                               bool multiple_layer_support,
                               FeatureLayer<FeatureType>* layer_ptr);

template <typename FeatureType>
bool loadFeatureBlocksFromFile(const std::string& file_path,
                               FeatureBlockMergingStrategy strategy,
                               FeatureLayer<FeatureType>* layer_ptr);

template <typename FeatureType>
bool loadFeatureBlocksFromStream(const size_t num_blocks,
                                 FeatureBlockMergingStrategy strategy,
                                 std::fstream* proto_file_ptr,
                                 FeatureLayer<FeatureType>* layer_ptr,
                                 uint32_t* tmp_byte_offset_ptr);

template <typename FeatureType>
bool loadFeatureLayer(const std::string& file_path,
                      const bool multiple_layer_support,
                      typename FeatureLayer<FeatureType>::Ptr* layer_ptr);
template <typename FeatureType>
bool loadFeatureLayer(const std::string& file_path,
                      typename FeatureLayer<FeatureType>::Ptr* layer_ptr);

template <typename FeatureType>
bool saveFeatureLayer(const FeatureLayer<FeatureType>& layer,
                      const std::string& file_path, bool clear_file = true);

template <typename FeatureType>
bool saveFeatureLayerSubset(const FeatureLayer<FeatureType>& layer,
                            const std::string& file_path,
                            const BlockIndexList& blocks_to_include,
                            bool include_all_blocks);

}  // namespace voxblox

#endif  // GLOBAL_FEATURE_MAP_FEATURE_UTILS_H_

#include "global_feature_map/feature_utils_inl.h"
