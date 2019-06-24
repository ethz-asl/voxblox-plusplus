#ifndef GLOBAL_SEGMENT_MAP_LABEL_TSDF_INTEGRATOR_H_
#define GLOBAL_SEGMENT_MAP_LABEL_TSDF_INTEGRATOR_H_

#include <map>
#include <vector>

#include <glog/logging.h>
#include <voxblox/alignment/icp.h>
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>
#include <voxblox/integrator/integrator_utils.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/utils/timing.h>

#include "global_segment_map/common.h"
#include "global_segment_map/icp_utils.h"
#include "global_segment_map/label_tsdf_map.h"
#include "global_segment_map/segment.h"
#include "global_segment_map/semantic_instance_label_fusion.h"

namespace voxblox {

class LabelTsdfIntegrator : public MergedTsdfIntegrator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef LongIndexHashMapType<AlignedVector<size_t>>::type VoxelMap;
  typedef VoxelMap::value_type VoxelMapElement;

  struct LabelTsdfConfig {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Label propagation parameters.
    // TODO(margaritaG): maybe use a relative measure, not absolue voxel count.
    // Minimum number of label voxels count for label propagation.
    size_t min_label_voxel_count = 20u;
    size_t max_num_icp_updates = 15u;
    // Truncation distance factor for label propagation.
    float label_propagation_td_factor = 1.0f;

    // Pairwise confidence-based segment merging logic.
    bool enable_pairwise_confidence_merging = true;
    float merging_min_overlap_ratio = 0.2f;
    int merging_min_frame_count = 30;

    // Semantic instance-aware segmentation.
    bool enable_semantic_instance_segmentation = false;

    // Object database logic.
    // Number of frames after which the updated
    // objects are published.
    int max_segment_age = 3;

    // Distance-based log-normal distribution of label confidence weights.
    bool enable_confidence_weight_dropoff = false;
    float lognormal_weight_mean = 0.0f;
    float lognormal_weight_sigma = 1.8f;
    float lognormal_weight_offset = 0.7f;

    // ICP params.
    bool enable_icp = false;
    bool keep_track_of_icp_correction = false;
  };

  LabelTsdfIntegrator(const Config& tsdf_config,
                      const LabelTsdfConfig& label_tsdf_config,
                      LabelTsdfMap* map);

  // Label propagation.
  void computeSegmentLabelCandidates(
      Segment* segment, std::map<Label, std::map<Segment*, size_t>>* candidates,
      std::map<Segment*, std::vector<Label>>* segment_merge_candidates,
      const std::set<Label>& assigned_labels = std::set<Label>());

  void decideLabelPointClouds(
      std::vector<voxblox::Segment*>* segments_to_integrate,
      std::map<voxblox::Label, std::map<voxblox::Segment*, size_t>>* candidates,
      std::map<Segment*, std::vector<Label>>* segment_merge_candidates);

  // Segment integration.
  void integratePointCloud(const Transformation& T_G_C,
                           const Pointcloud& points_C, const Colors& colors,
                           const Label& label, const bool freespace_points);

  // Segment merging.
  // Not thread safe.
  void mergeLabels(LLSet* merges_to_publish);

  // Object database.
  void getLabelsToPublish(
      std::vector<voxblox::Label>* segment_labels_to_publish);

  Transformation getIcpRefined_T_G_C(const Transformation& T_G_C_init,
                                     const Pointcloud& point_cloud);

 protected:
  // Label propagation.
  // Fetch the next segment label pair which has overall
  // the highest voxel count.
  bool getNextSegmentLabelPair(
      const std::set<Segment*>& labelled_segments,
      std::set<Label>* assigned_labels,
      std::map<voxblox::Label, std::map<voxblox::Segment*, size_t>>* candidates,
      std::map<Segment*, std::vector<Label>>* segment_merge_candidates,
      std::pair<Segment*, Label>* segment_label_pair);

  Label getNextUnassignedLabel(const LabelVoxel& voxel,
                               const std::set<Label>& assigned_labels);

  void checkForSegmentLabelMergeCandidate(
      const Label& label, const int label_points_count,
      const int segment_points_count,
      std::unordered_set<Label>* merge_candidate_labels);

  void increaseLabelCountForSegment(
      Segment* segment, const Label& label, const int segment_points_count,
      std::map<Label, std::map<Segment*, size_t>>* candidates,
      std::unordered_set<Label>* merge_candidate_labels);

  void increasePairwiseConfidenceCount(
      const std::vector<Label>& merge_candidates);

  void updateVoxelLabelAndConfidence(LabelVoxel* label_voxel,
                                     const Label& preferred_label = 0u);

  void addVoxelLabelConfidence(const Label& label,
                               const LabelConfidence& confidence,
                               LabelVoxel* label_voxel);

  void increaseLabelFramesCount(const Label& label);

  // Increase or decrease the voxel count for a label.
  void changeLabelCount(const Label& label, const int count);

  // Will return a pointer to a voxel located at global_voxel_idx in the label
  // layer. Thread safe.
  // Takes in the last_block_idx and last_block to prevent unneeded map
  // lookups. If the block this voxel would be in has not been allocated, a
  // block in temp_label_block_map_ is created/accessed and a voxel from this
  // map is returned instead. Unlike the layer, accessing
  // temp_label_block_map_ is controlled via a mutex allowing it to grow
  // during integration. These temporary blocks can be merged into the layer
  // later by calling updateLayerWithStoredBlocks()
  LabelVoxel* allocateStorageAndGetLabelVoxelPtr(
      const GlobalIndex& global_voxel_idx, Block<LabelVoxel>::Ptr* last_block,
      BlockIndex* last_block_idx);

  // NOT thread safe
  void updateLabelLayerWithStoredBlocks();

  // Updates label_voxel. Thread safe.
  void updateLabelVoxel(const Point& point_G, const Label& label,
                        LabelVoxel* label_voxel,
                        const LabelConfidence& confidence = 1u);

  void integrateVoxel(
      const Transformation& T_G_C, const Pointcloud& points_C,
      const Colors& colors, const Label& label, const bool enable_anti_grazing,
      const bool clearing_ray,
      const VoxelMapElement& global_voxel_idx_to_point_indices,
      const LongIndexHashMapType<AlignedVector<size_t>>::type& voxel_map);

  void integrateVoxels(
      const Transformation& T_G_C, const Pointcloud& points_C,
      const Colors& colors, const Label& label, const bool enable_anti_grazing,
      const bool clearing_ray,
      const LongIndexHashMapType<AlignedVector<size_t>>::type& voxel_map,
      const LongIndexHashMapType<AlignedVector<size_t>>::type& clear_map,
      const size_t thread_idx);

  void integrateRays(
      const Transformation& T_G_C, const Pointcloud& points_C,
      const Colors& colors, const Label& label, const bool enable_anti_grazing,
      const bool clearing_ray,
      const LongIndexHashMapType<AlignedVector<size_t>>::type& voxel_map,
      const LongIndexHashMapType<AlignedVector<size_t>>::type& clear_map);

  FloatingPoint computeConfidenceWeight(const FloatingPoint& distance);

  // Not thread safe.
  void swapLabels(const Label& old_label, const Label& new_label);

  inline void clearCurrentFrameInstanceLabels() {
    current_to_global_instance_map_.clear();
  }

  void resetCurrentFrameUpdatedLabelsAge();

  void addPairwiseConfidenceCount(const LLMapIt& label_map_it,
                                  const Label& label, const int count);

  void adjustPairwiseConfidenceAfterMerging(const Label& new_label,
                                            const Label& old_label);

  bool getNextMerge(Label* new_label, Label* old_label);

  inline Label getFreshLabel() {
    CHECK_LT(*highest_label_ptr_, std::numeric_limits<unsigned short>::max());
    return ++(*highest_label_ptr_);
  }

  inline InstanceLabel getFreshInstance() {
    CHECK_LT(*highest_instance_ptr_,
             std::numeric_limits<unsigned short>::max());
    return ++(*highest_instance_ptr_);
  }

  // Label layer.
  LabelTsdfConfig label_tsdf_config_;
  Layer<LabelVoxel>* label_layer_;

  // Temporary block storage, used to hold blocks that need to be created
  // while integrating a new pointcloud.
  std::mutex temp_label_block_mutex_;
  Layer<LabelVoxel>::BlockHashMap temp_label_block_map_;

  // We need to prevent simultaneous access to the voxels in the map. We
  // could
  // put a single mutex on the map or on the blocks, but as voxel updating
  // is
  // the most expensive operation in integration and most voxels are close
  // together, both strategies would bottleneck the system. We could make a
  // mutex per voxel, but this is too ram heavy as one mutex = 40 bytes.
  // Because of this we create an array that is indexed by the first n bits
  // of
  // the voxels hash. Assuming a uniform hash distribution, this means the
  // chance of two threads needing the same lock for unrelated voxels is
  // (num_threads / (2^n)). For 8 threads and 12 bits this gives 0.2%.
  ApproxHashArray<12, std::mutex, GlobalIndex, LongIndexHash> mutexes_;

  Label* highest_label_ptr_;
  LMap* label_count_map_ptr_;
  std::mutex updated_labels_mutex_;
  std::set<Label> updated_labels_;

  // Pairwise confidence merging.
  LLMap pairwise_confidence_;

  // ICP variables.
  std::shared_ptr<ICP> icp_;
  Transformation T_Gicp_G_;

  // Semantic instance-aware segmentation.
  SemanticInstanceLabelFusion* semantic_instance_label_fusion_ptr_;
  InstanceLabel* highest_instance_ptr_;
  std::map<SemanticLabel, SemanticLabel> current_to_global_instance_map_;

  // Object database.
  LMap labels_to_publish_;
};

}  // namespace voxblox

#endif  // GLOBAL_SEGMENT_MAP_LABEL_TSDF_INTEGRATOR_H_
