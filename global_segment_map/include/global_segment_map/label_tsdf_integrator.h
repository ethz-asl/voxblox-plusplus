#ifndef GLOBAL_SEGMENT_MAP_LABEL_TSDF_INTEGRATOR_H_
#define GLOBAL_SEGMENT_MAP_LABEL_TSDF_INTEGRATOR_H_

#include <algorithm>
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
};

class LabelTsdfIntegrator : public TsdfIntegrator {
 public:
  LabelTsdfIntegrator(const Config& config, Layer<TsdfVoxel>* tsdf_layer,
                      Layer<LabelVoxel>* label_layer, Label* highest_label)
      : TsdfIntegrator(config, CHECK_NOTNULL(tsdf_layer)),
        label_layer_(CHECK_NOTNULL(label_layer)),
        highest_label_(CHECK_NOTNULL(highest_label)) {
    CHECK(label_layer_);

    // TODO(grinvalm): parametrize.
    pairwise_confidence_merging_ = true;
    pairwise_confidence_ratio_threshold_ = 0.05;
    pairwise_confidence_threshold_ = 2;

    // Experiments showed that capped confidence value
    // only introduces artifacts in planar regions.
    cap_confidence_ = false;
    confidence_cap_value_ = 10;
  }

  inline void computeSegmentLabelCandidates(Segment* segment,
      std::map<Label, std::map<Segment*, size_t> >* candidates) {
    // Flag to check whether there exists at least one label candidate.
    bool candidate_label_exists = false;
    int segment_points_count = segment->points_C_.size();
    std::unordered_set<Label> merge_candidate_labels;

    for (const Point& point_C : segment->points_C_) {
      const Point point_G = segment->T_G_C_ * point_C;

      // Get the corresponding voxel by 3D position in world frame.
      Layer<LabelVoxel>::BlockType::ConstPtr block_ptr =
          label_layer_->getBlockPtrByCoordinates(point_G);

      if (block_ptr != nullptr) {
        candidate_label_exists = true;
        const LabelVoxel& voxel = block_ptr->getVoxelByCoordinates(point_G);
        // Allocated but unobserved voxels have label 0
        if (voxel.label != 0u) {
          auto label_it = candidates->find(voxel.label);
          if (label_it != candidates->end()) {
            auto segment_it = label_it->second.find(segment);
            if (segment_it != label_it->second.end()) {
              ++segment_it->second;

              if (pairwise_confidence_merging_) {
                // Add all segment labels that overlap with more than
                // a threshold ratio to the set of potential merge candidates.
                if ((float)segment_it->second / (float)segment_points_count >
                                        pairwise_confidence_ratio_threshold_) {
                  if (merge_candidate_labels.find(voxel.label)
                      == merge_candidate_labels.end()) {
                    merge_candidate_labels.insert(voxel.label);
                  }
                }
              }
            } else {
              label_it->second.insert(std::make_pair(segment, 1u));
            }
          } else {
            std::map<Segment*, size_t> segment_points_count;
            segment_points_count.insert(std::make_pair(segment, 1u));
            candidates->insert(std::make_pair(voxel.label, segment_points_count));
          }
        }
      }
    }

    if (pairwise_confidence_merging_) {
      std::vector<Label> merge_candidates;
      std::copy(merge_candidate_labels.begin(), merge_candidate_labels.end(),
                std::back_inserter(merge_candidates));

      // For every couple of labels from the merge candidates
      // set or increase their pairwise confidence.
      for (int i = 0; i < merge_candidates.size(); i++) {
        for (int j = i+1; j < merge_candidates.size(); j++) {
          Label label1 = merge_candidates[i];
          Label label2 = merge_candidates[j];

          if (label1 != label2) {
            if (label1 > label2) {
              Label tmp = label2;
              label2 = label1;
              label1 = tmp;
            }
            auto pairs = pairwise_confidence_.find(label1);
            if (pairs != pairwise_confidence_.end()) {
              auto confidence = pairs->second.find(label2);
              if (confidence != pairs->second.end()) {
                ++confidence->second;
                LOG(INFO) << "Confidence between label " << label1
                    << " and label " << label2 << " is " << confidence->second;
              } else {
                pairs->second.insert(std::make_pair(label2, 1));
                LOG(INFO) << "Confidence between label " << label1
                    << " and label " << label2 << " is 1";
              }
            } else {
              std::map<Label, int> confidence_pair;
              confidence_pair.insert(std::make_pair(label2, 1));
              pairwise_confidence_.insert(std::make_pair(label1, confidence_pair));
              LOG(INFO) << "Confidence between label " << label1
                  << " and label " << label2 << " is 1";
            }
          }
        }
      }
    }

    // Previously unobserved segment get an unseen label.
    if (!candidate_label_exists) {
      Label fresh_label = getFreshLabel();
      std::map<Segment*, size_t> map;
      map.insert(std::pair<Segment*, size_t>(segment, segment->points_C_.size()));
      candidates->insert(std::pair<Label, std::map<Segment*, size_t> >
                                                            (fresh_label, map));
    }

//    // Print candidates
//    ROS_INFO("Candidates now: ");
//    for(auto it = candidates->cbegin(); it != candidates->cend(); ++it) {
//        std::cout << "Label: " << it->first << "\n";
//        for(auto itt = it->second.cbegin(); itt != it->second.cend(); ++itt) {
//            std::cout <<"Segment: " <<  itt->first << ", count: "
//                      << itt->second << "\n";
//        }
//    }
  }

  // Fetch the next segment label pair which has overall
  // the highest voxel count.
  inline bool getNextSegmentLabelPair(
      std::map<voxblox::Label, std::map<voxblox::Segment*, size_t> >* candidates,
      std::set<Segment*>* labelled_segments,
      std::pair<Segment*, Label>* segment_label_pair) {
    Label max_label;
    size_t max_count = 0;
    Segment* max_segment;

    for (auto label_it = candidates->begin();
        label_it != candidates->end(); ++label_it) {
      for (auto segment_it = label_it->second.begin();
          segment_it != label_it->second.end(); segment_it++) {
        if (segment_it->second > max_count
            && labelled_segments->find(segment_it->first)
                == labelled_segments->end()) {
          max_segment = segment_it->first;
          max_count = segment_it->second;
          max_label = label_it->first;
        }
      }
    }
    if (max_count == 0) {
      return false;
    }
    segment_label_pair->first = max_segment;
    segment_label_pair->second = max_label;

    return true;
  }

  inline void decideLabelPointClouds(
      std::vector<voxblox::Segment*>* segments_to_integrate,
      std::map<voxblox::Label, std::map<voxblox::Segment*, size_t> >* candidates) {
    std::set<Segment*> labelled_segments;
    std::pair< Segment*, Label> pair;

    while (getNextSegmentLabelPair(candidates, &labelled_segments, &pair)) {
      for (size_t i = 0; i < pair.first->points_C_.size(); ++i) {
        pair.first->labels_.push_back(pair.second);
      }
      labelled_segments.insert(pair.first);
      candidates->erase(pair.second);
    }

    // For every segment that didn't get a label, assign it an unseen label.
    for (auto segment_it = segments_to_integrate->begin();
        segment_it != segments_to_integrate->end(); ++segment_it) {
      if (labelled_segments.find(*segment_it) == labelled_segments.end()) {
        Label fresh = getFreshLabel();
        for (size_t i = 0; i <(*segment_it)->points_C_.size(); ++i) {
          (*segment_it)->labels_.push_back(fresh);
        }
        labelled_segments.insert(*segment_it);
      }
    }
  }

  // Increase or decrease the voxel count for a label.
  // TODO(grinvalm): when count for a label goes to 0
  // remove label form label_count.
  inline void changeLabelCount(const Label label, int count) {
    auto label_count = label_count_.find(label);
    if (label_count != label_count_.end()) {
      label_count->second = label_count->second + count;
    } else {
      if (label != 0u) {
        label_count_.insert(std::make_pair(label, count));
      }
    }
  }

  inline void updateLabelVoxel(const Label& label,
                               LabelVoxel* label_voxel) {
    updateLabelVoxel(label, 1, label_voxel);
  }

  inline void updateLabelVoxel(const Label& label,
                               const float confidence,
                               LabelVoxel* label_voxel) {
    CHECK_NOTNULL(label_voxel);

    if (label_voxel->label == label) {
      label_voxel->label_confidence = label_voxel->label_confidence + confidence;
      if (cap_confidence_) {
        if (label_voxel->label_confidence > confidence_cap_value_) {
          label_voxel->label_confidence = confidence_cap_value_;
        }
      }
    } else {
      if (label_voxel->label_confidence <= 0.0f) {
        changeLabelCount(label, 1);
        changeLabelCount(label_voxel->label, -1);

        label_voxel->label = label;
        label_voxel->label_confidence = confidence;
        if (*highest_label_ < label) {
          *highest_label_ = label;
        }
      } else {
        label_voxel->label_confidence = label_voxel->label_confidence - confidence;
      }
    }
  }

  inline void bundleRays(
      const Transformation& T_G_C, const Pointcloud& points_C,
      BlockHashMapType<std::vector<size_t>>::type* voxel_map,
      BlockHashMapType<std::vector<size_t>>::type* clear_map) {
    for (size_t pt_idx = 0; pt_idx < points_C.size(); ++pt_idx) {
      const Point& point_C = points_C[pt_idx];
      const Point point_G = T_G_C * point_C;

      FloatingPoint ray_distance = (point_C).norm();
      if (ray_distance < config_.min_ray_length_m) {
        continue;
      } else if (config_.allow_clear &&
                 ray_distance > config_.max_ray_length_m) {
        VoxelIndex voxel_index =
            getGridIndexFromPoint(point_G, voxel_size_inv_);
        (*clear_map)[voxel_index].push_back(pt_idx);
        continue;
      }

      // Figure out what the end voxel is here.
      VoxelIndex voxel_index = getGridIndexFromPoint(point_G, voxel_size_inv_);
      (*voxel_map)[voxel_index].push_back(pt_idx);
    }

    LOG(INFO) << "Went from " << points_C.size() << " points to "
              << voxel_map->size() << " raycasts  and " << clear_map->size()
              << " clear rays.";
  }

  void integratePointCloudMerged(const Transformation& T_G_C,
                                 const Pointcloud& points_C,
                                 const Colors& colors,
                                 const Labels& labels,
                                 bool discard) {
    DCHECK_EQ(points_C.size(), colors.size());

    timing::Timer integrate_timer("integrate");

    const Point& origin = T_G_C.getPosition();

    // Pre-compute a list of unique voxels to end on.
    // Create a hashmap: VOXEL INDEX -> index in original cloud.
    BlockHashMapType<std::vector<size_t>>::type voxel_map;
    // This is a hash map (same as above) to all the indices that need to be
    // cleared.
    BlockHashMapType<std::vector<size_t>>::type clear_map;

    bundleRays(T_G_C, points_C, &voxel_map, &clear_map);

    const Point voxel_center_offset(0.5, 0.5, 0.5);

    FloatingPoint truncation_distance = config_.default_truncation_distance;
    for (const BlockHashMapType<std::vector<size_t>>::type::value_type& kv :
         voxel_map) {
      if (kv.second.empty()) {
        continue;
      }
      // Key actually doesn't matter at all.
      Point mean_point_C = Point::Zero();
      Color mean_color;

      Label mean_label;
      float label_confidence;

      float total_weight = 0.0;

      for (size_t pt_idx : kv.second) {
        const Point& point_C = points_C[pt_idx];
        const Color& color = colors[pt_idx];

        const Label& label = labels[pt_idx];
        if (label == 0u) {
          LOG(ERROR) << "Integrating a 0 label";
        }

        float point_weight = getVoxelWeight(
            point_C, T_G_C * point_C, origin,
            (kv.first.cast<FloatingPoint>() + voxel_center_offset) *
                voxel_size_);
        mean_point_C = (mean_point_C * total_weight + point_C * point_weight) /
                       (total_weight + point_weight);
        mean_color = Color::blendTwoColors(mean_color, total_weight, color,
                                           point_weight);
        // Assuming for a segment pointcloud all labels are same so for
        // the voxel label it is enough to take the label of the last point
        mean_label = label;
        label_confidence++;

        total_weight += point_weight;
      }

      const Point point_G = T_G_C * mean_point_C;
      const Ray unit_ray = (point_G - origin).normalized();
      const Point ray_end = point_G + unit_ray * truncation_distance;
      const Point ray_start = config_.voxel_carving_enabled
                                  ? origin
                                  : (point_G - unit_ray * truncation_distance);

      const Point start_scaled = ray_start * voxel_size_inv_;
      const Point end_scaled = ray_end * voxel_size_inv_;

      IndexVector global_voxel_index;
      timing::Timer cast_ray_timer("integrate/cast_ray");
      castRay(start_scaled, end_scaled, &global_voxel_index);
      cast_ray_timer.Stop();

      timing::Timer update_voxels_timer("integrate/update_voxels");

      BlockIndex last_block_idx = BlockIndex::Zero();
      Block<TsdfVoxel>::Ptr block;
      Block<LabelVoxel>::Ptr label_block;

      for (const AnyIndex& global_voxel_idx : global_voxel_index) {
        if (discard) {
          // Check if this one is already the the block hash map for this
          // insertion. Skip this to avoid grazing.
          if (global_voxel_idx != kv.first &&
              voxel_map.find(global_voxel_idx) != voxel_map.end()) {
            continue;
          }
        }

        BlockIndex block_idx = getGridIndexFromPoint(
            global_voxel_idx.cast<FloatingPoint>(), voxels_per_side_inv_);

        VoxelIndex local_voxel_idx =
            getLocalFromGlobalVoxelIndex(global_voxel_idx, voxels_per_side_);

        if (!block || block_idx != last_block_idx) {
           if (!block != !label_block) {
             LOG(FATAL) << "Block allocation differs between the two layers.";
           }
           block = layer_->allocateBlockPtrByIndex(block_idx);
           label_block = label_layer_->allocateBlockPtrByIndex(block_idx);

           block->updated() = true;
           label_block->updated() = true;

           last_block_idx = block_idx;
         }

        const Point voxel_center_G =
            block->computeCoordinatesFromVoxelIndex(local_voxel_idx);
        TsdfVoxel& tsdf_voxel = block->getVoxelByVoxelIndex(local_voxel_idx);
        LabelVoxel& label_voxel =
                   label_block->getVoxelByVoxelIndex(local_voxel_idx);

        updateTsdfVoxel(origin, mean_point_C, point_G, voxel_center_G,
                        mean_color, truncation_distance, total_weight,
                        &tsdf_voxel);
        // TODO(grinvalm): change the confidnce logic
        // to exploit the label_confidence count here
//        updateLabelVoxel(mean_label, label_confidence, &label_voxel);
        updateLabelVoxel(mean_label, 1, &label_voxel);
      }
      update_voxels_timer.Stop();
    }

    timing::Timer clear_timer("integrate/clear");
    BlockIndex last_block_idx = BlockIndex::Zero();
    Block<TsdfVoxel>::Ptr block;
    Block<LabelVoxel>::Ptr label_block;
    for (const BlockHashMapType<std::vector<size_t>>::type::value_type& kv :
         clear_map) {
      if (kv.second.empty()) {
        continue;
      }
      // Key actually doesn't matter at all.
      Point point_C = Point::Zero();
      Color color;
      Label label;
      float weight = 0.0;

      for (size_t pt_idx : kv.second) {
        // Just take first.
        point_C = points_C[pt_idx];
        color = colors[pt_idx];
        label = labels[pt_idx];

        weight = getVoxelWeight(
            point_C, T_G_C * point_C, origin,
            (kv.first.cast<FloatingPoint>() + voxel_center_offset) *
                voxel_size_);
        break;
      }

      const Point point_G = T_G_C * point_C;
      const Ray unit_ray = (point_G - origin).normalized();
      const Point ray_end = origin + unit_ray * config_.max_ray_length_m;
      const Point ray_start = origin;

      const Point start_scaled = ray_start * voxel_size_inv_;
      const Point end_scaled = ray_end * voxel_size_inv_;

      IndexVector global_voxel_index;
      timing::Timer cast_ray_timer("integrate/cast_ray");
      castRay(start_scaled, end_scaled, &global_voxel_index);
      cast_ray_timer.Stop();

      for (const AnyIndex& global_voxel_idx : global_voxel_index) {
        if (discard) {
          // Check if this one is already the the block hash map for this
          // insertion. Skip this to avoid grazing.
          if (voxel_map.find(global_voxel_idx) != voxel_map.end()) {
            continue;
          }
        }

        BlockIndex block_idx = getGridIndexFromPoint(
            global_voxel_idx.cast<FloatingPoint>(), voxels_per_side_inv_);

        VoxelIndex local_voxel_idx =
            getLocalFromGlobalVoxelIndex(global_voxel_idx, voxels_per_side_);


        if (!block || block_idx != last_block_idx) {
          if (!block != !label_block) {
            LOG(FATAL) << "Block allocation differs between the two layers.";
          }
          block = layer_->allocateBlockPtrByIndex(block_idx);
          label_block = label_layer_->allocateBlockPtrByIndex(block_idx);

          block->updated() = true;
          label_block->updated() = true;

          last_block_idx = block_idx;
        }

        const Point voxel_center_G =
            block->computeCoordinatesFromVoxelIndex(local_voxel_idx);
        TsdfVoxel& tsdf_voxel = block->getVoxelByVoxelIndex(local_voxel_idx);
        LabelVoxel& label_voxel =
            label_block->getVoxelByVoxelIndex(local_voxel_idx);

        updateTsdfVoxel(origin, point_C, point_G, voxel_center_G, color,
                        truncation_distance, weight, &tsdf_voxel);
        updateLabelVoxel(label, &label_voxel);
      }
    }

    clear_timer.Stop();

    integrate_timer.Stop();
  }

  void swapLabels(Label old_label, Label new_label) {
    BlockIndexList all_label_blocks;
    label_layer_->getAllAllocatedBlocks(&all_label_blocks);

    for (const BlockIndex& block_index : all_label_blocks) {
      Block<LabelVoxel>::Ptr block =
                      label_layer_->getBlockPtrByIndex(block_index);
      size_t vps = block->voxels_per_side();
      for (int i = 0; i < vps * vps * vps; i++) {
        LabelVoxel& voxel = block->getVoxelByLinearIndex(i);
        if (voxel.label == old_label) {
          voxel.label = new_label;
          changeLabelCount(new_label, 1);
          changeLabelCount(old_label, -1);

          block->updated() = true;
        }
      }
    }
  }

  void mergeLabels() {
    if (pairwise_confidence_merging_) {
      for (auto& confidence_map: pairwise_confidence_) {
        for (auto confidence_pair: confidence_map.second) {
          if (confidence_pair.second > pairwise_confidence_threshold_) {
            swapLabels(confidence_map.first, confidence_pair.first);
            LOG(ERROR) << "Merging labels " << confidence_map.first
                       << " and " << confidence_pair.first;
            confidence_map.second.erase(confidence_pair.first);
          }
        }
      }
    }
  }

  Label getFreshLabel() {
    CHECK_LT(*highest_label_, 0xFFFFFFFF);
    return ++(*highest_label_);
  }

  // Get the list of all labels
  // for which the voxel count is greater than 0.
  std::vector<Label> getLabelsList() {
    std::vector<Label> labels;
    for (auto& label_count_pair: label_count_) {
      if (label_count_pair.second > 0) {
        labels.push_back(label_count_pair.first);
      }
    }
    return labels;
  }

 protected:
  Layer<LabelVoxel>* label_layer_;

  Label* highest_label_;
  std::map<Label, int> label_count_;

  // Pairwise confidence merging settings.
  std::map<Label, std::map<Label, int> > pairwise_confidence_;
  bool pairwise_confidence_merging_;
  float pairwise_confidence_ratio_threshold_;
  int pairwise_confidence_threshold_;

  // Cap confidence settings.
  bool cap_confidence_;
  int confidence_cap_value_;
};

}  // namespace voxblox

#endif  // GLOBAL_SEGMENT_MAP_LABEL_TSDF_INTEGRATOR_H_
