#ifndef GLOBAL_SEGMENT_MAP_LABEL_TSDF_INTEGRATOR_H_
#define GLOBAL_SEGMENT_MAP_LABEL_TSDF_INTEGRATOR_H_

#include <algorithm>
#include <list>
#include <map>
#include <utility>
#include <vector>

#include <glog/logging.h>
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>
#include <voxblox/integrator/integrator_utils.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/utils/timing.h>

#include "global_segment_map/label_tsdf_map.h"
#include "global_segment_map/label_voxel.h"

namespace voxblox {

class Segment {
 public:
  voxblox::Pointcloud points_C_;
  voxblox::Transformation T_G_C_;
  voxblox::Colors colors_;
  voxblox::Labels labels_;
  voxblox::SemanticLabel semantic_label_;
  voxblox::InstanceLabel instance_label_;
};

class LabelTsdfIntegrator : public MergedTsdfIntegrator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef LongIndexHashMapType<AlignedVector<size_t>>::type VoxelMap;
  typedef VoxelMap::value_type VoxelMapElement;
  typedef std::map<Label, int> LMap;
  typedef std::map<Label, int>::iterator LMapIt;
  typedef std::map<SemanticLabel, int> SLMap;
  typedef std::map<Label, LMap> LLMap;
  typedef std::map<Label, LMap>::iterator LLMapIt;
  typedef std::map<Label, SLMap> LSLMap;
  typedef std::set<Label> LSet;
  typedef std::set<Label>::iterator LSetIt;
  typedef std::map<Label, LSet> LLSet;
  typedef std::map<Label, LSet>::iterator LLSetIt;

  struct LabelTsdfConfig {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // TODO(margaritaG): maybe use a relative measure, not absolue voxel count.
    // Minimum number of label voxels count for label propagation.
    size_t min_label_voxel_count = 20;
    // Factor determining the label propagation truncation distance.
    float label_propagation_td_factor = 1.0;

    // Pairwise confidence-based segment merging logic.
    bool enable_pairwise_confidence_merging = true;
    float pairwise_confidence_ratio_threshold = 0.2f;
    int pairwise_confidence_count_threshold = 30;

    // Object database logic.
    // Number of frames after which the updated
    // objects are published.
    int segment_flushing_age_threshold = 3;

    // Distance-based log-normal distribution of label confidence weights.
    bool enable_confidence_weight_dropoff = false;
    float lognormal_weight_mean = 0.0f;
    float lognormal_weight_sigma = 1.8f;
    float lognormal_weight_offset = 0.7f;
  };

  LabelTsdfIntegrator(const Config& config,
                      const LabelTsdfConfig& label_tsdf_config,
                      Layer<TsdfVoxel>* tsdf_layer,
                      Layer<LabelVoxel>* label_layer, Label* highest_label,
                      InstanceLabel* highest_instance);

  void checkForSegmentLabelMergeCandidate(
      Label label, int label_points_count, int segment_points_count,
      std::unordered_set<Label>* merge_candidate_labels);

  void increaseLabelCountForSegment(
      Segment* segment, Label label, int segment_points_count,
      std::map<Label, std::map<Segment*, size_t>>* candidates,
      std::unordered_set<Label>* merge_candidate_labels);

  void increasePairwiseConfidenceCount(std::vector<Label> merge_candidates);

  Label getNextUnassignedLabel(const LabelVoxel& voxel,
                               const std::set<Label>& assigned_labels);

  void updateVoxelLabelAndConfidence(LabelVoxel* label_voxel,
                                     const Label& preferred_label = 0u);

  void addVoxelLabelConfidence(const Label& label,
                               const LabelConfidence& confidence,
                               LabelVoxel* label_voxel);

  void computeSegmentLabelCandidates(
      Segment* segment, std::map<Label, std::map<Segment*, size_t>>* candidates,
      std::map<Segment*, std::vector<Label>>* segment_merge_candidates,
      const std::set<Label>& assigned_labels = std::set<Label>());

  // Fetch the next segment label pair which has overall
  // the highest voxel count.
  bool getNextSegmentLabelPair(
      const std::set<Segment*>& labelled_segments,
      std::set<Label>* assigned_labels,
      std::map<voxblox::Label, std::map<voxblox::Segment*, size_t>>* candidates,
      std::map<Segment*, std::vector<Label>>* segment_merge_candidates,
      std::pair<Segment*, Label>* segment_label_pair);

  void decideLabelPointClouds(
      std::vector<voxblox::Segment*>* segments_to_integrate,
      std::map<voxblox::Label, std::map<voxblox::Segment*, size_t>>* candidates,
      std::map<Segment*, std::vector<Label>>* segment_merge_candidates);

  inline SemanticLabel getLabelInstance(const Label& label) {
    std::set<SemanticLabel> assigned_instances;
    return getLabelInstance(label, assigned_instances);
  }

  SemanticLabel getLabelInstance(const Label& label,
                                 std::set<SemanticLabel>& assigned_instances);

  void increaseLabelFramesCount(const Label& label);

  // Increase or decrease the voxel count for a label.
  void changeLabelCount(const Label label, int count);

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

  void increaseLabelInstanceCount(const Label& label,
                                  const SemanticLabel& instance_label);

  void decreaseLabelInstanceCount(const Label& label,
                                  const SemanticLabel& instance_label);

  void increaseLabelClassCount(const Label& label,
                               const SemanticLabel& semantic_label);

  // Updates label_voxel. Thread safe.
  void updateLabelVoxel(const Point& point_G, const Label& label,
                        LabelVoxel* label_voxel,
                        const LabelConfidence& confidence = 1u);

  void integratePointCloud(const Transformation& T_G_C,
                           const Pointcloud& points_C, const Colors& colors,
                           const Labels& labels, const bool freespace_points);

  void integrateVoxel(const Transformation& T_G_C, const Pointcloud& points_C,
                      const Colors& colors, const Labels& labels,
                      const bool enable_anti_grazing, const bool clearing_ray,
                      const VoxelMapElement& global_voxel_idx_to_point_indices,
                      const VoxelMap& voxel_map);

  void integrateVoxels(const Transformation& T_G_C, const Pointcloud& points_C,
                       const Colors& colors, const Labels& labels,
                       const bool enable_anti_grazing, const bool clearing_ray,
                       const VoxelMap& voxel_map, const VoxelMap& clear_map,
                       const size_t thread_idx);

  void integrateRays(const Transformation& T_G_C, const Pointcloud& points_C,
                     const Colors& colors, const Labels& labels,
                     const bool enable_anti_grazing, const bool clearing_ray,
                     const VoxelMap& voxel_map, const VoxelMap& clear_map);

  FloatingPoint computeConfidenceWeight(FloatingPoint distance);

  // Not thread safe.
  void swapLabels(Label old_label, Label new_label);

  inline void clearCurrentFrameInstanceLabels() {
    current_to_global_instance_map_.clear();
  }

  void resetCurrentFrameUpdatedLabelsAge();

  void getLabelsToPublish(
      std::vector<voxblox::Label>* segment_labels_to_publish);

  inline LMap* getLabelsAgeMapPtr() { return &labels_to_publish_; }

  inline LSLMap* getLabelClassCountPtr() { return &label_class_count_; }

  inline LSLMap* getLabelInstanceCountPtr() { return &label_instance_count_; }

  inline LMap* getLabelsFrameCountPtr() { return &label_frames_count_; }

  void addPairwiseConfidenceCount(LLMapIt label_map_it, Label label, int count);

  void adjustPairwiseConfidenceAfterMerging(const Label& new_label,
                                            const Label& old_label);

  bool getNextMerge(Label* new_label, Label* old_label);

  // Not thread safe.
  void mergeLabels(LLSet* merges_to_publish);

  Label getFreshLabel() {
    CHECK_LT(*highest_label_, std::numeric_limits<unsigned short>::max());
    return ++(*highest_label_);
  }

  InstanceLabel getFreshInstance() {
    CHECK_LT(*highest_instance_, std::numeric_limits<unsigned char>::max());
    return ++(*highest_instance_);
  }

  // Get the list of all labels
  // for which the voxel count is greater than 0.
  std::vector<Label> getLabelsList();

 protected:
  LabelTsdfConfig label_tsdf_config_;
  Layer<LabelVoxel>* label_layer_;

  // Temporary block storage, used to hold blocks that need to be created
  // while integrating a new pointcloud.
  std::mutex temp_label_block_mutex_;
  Layer<LabelVoxel>::BlockHashMap temp_label_block_map_;

  Label* highest_label_;
  LMap labels_count_map_;

  // Pairwise confidence merging.
  LLMap pairwise_confidence_;

  // Per frame voxel count of semantic label.
  LSLMap label_class_count_;

  InstanceLabel* highest_instance_;
  std::map<SemanticLabel, SemanticLabel> current_to_global_instance_map_;
  LSLMap label_instance_count_;
  LMap label_frames_count_;

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

  std::mutex updated_labels_mutex_;
  std::set<Label> updated_labels_;

  LMap labels_to_publish_;
};  // namespace voxblox

}  // namespace voxblox

#endif  // GLOBAL_SEGMENT_MAP_LABEL_TSDF_INTEGRATOR_H_
