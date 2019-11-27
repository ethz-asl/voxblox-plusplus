#include "global_segment_map/utils/label_voxel_utils.h"

namespace voxblox {
namespace utils {

void updateVoxelLabelAndConfidence(LabelVoxel* label_voxel,
                                   const Label& preferred_label) {
  CHECK_NOTNULL(label_voxel);
  Label max_label = 0u;
  LabelConfidence max_confidence = 0u;
  for (const LabelCount& label_count : label_voxel->label_count) {
    if (label_count.label_confidence > max_confidence ||
        (label_count.label == preferred_label && preferred_label != 0u &&
         label_count.label_confidence == max_confidence)) {
      max_confidence = label_count.label_confidence;
      max_label = label_count.label;
    }
  }
  label_voxel->label = max_label;
  label_voxel->label_confidence = max_confidence;
}

void addVoxelLabelConfidence(const Label& label,
                             const LabelConfidence& confidence,
                             LabelVoxel* label_voxel) {
  CHECK_NOTNULL(label_voxel);
  bool updated = false;
  for (LabelCount& label_count : label_voxel->label_count) {
    if (label_count.label == label) {
      // Label already observed in this voxel.
      label_count.label_confidence = label_count.label_confidence + confidence;
      updated = true;
      break;
    }
  }
  if (updated == false) {
    for (LabelCount& label_count : label_voxel->label_count) {
      if (label_count.label == 0u) {
        // This is the first allocated but unused index in the map
        // in which the new entry should be added.
        label_count.label = label;
        label_count.label_confidence = confidence;
        updated = true;
        break;
      }
    }
  }
  if (updated == false) {
    // TODO(margaritaG): handle this nicely or remove.
    // LOG(FATAL) << "Out-of-memory for storing labels and confidences for this
    // "
    //               " voxel. Please increse size of array.";
  }
}

template <>
bool isObservedVoxel(const LabelVoxel& voxel) {
  return voxel.label != 0u;
}

}  // namespace utils

template <>
void mergeVoxelAIntoVoxelB(const LabelVoxel& voxel_A, LabelVoxel* voxel_B) {
  CHECK_NOTNULL(voxel_B);
  // voxel_B->label_count[0].label = 2u;
  // voxel_B->label_count[0].label_confidence = 10;
  // voxel_B->label = 2u;
  // voxel_B->label_confidence = 10;

  if (voxel_A.label == 0u) {
    // No new observations to be merged, voxel B stays as is.
    return;
  }

  int label_count_size =
      sizeof(voxel_B->label_count) / sizeof(*voxel_B->label_count);

  if (voxel_B->label == 0u) {
    // Simply copy voxel A into voxel B.
    for (int i = 0; i < label_count_size; ++i) {
      voxel_B->label_count[i].label = voxel_A.label_count[i].label;
      voxel_B->label_count[i].label_confidence =
          voxel_A.label_count[i].label_confidence;
    }
    voxel_B->label = voxel_A.label;
    voxel_B->label_confidence = voxel_A.label_confidence;
  }

  else {
    // Both voxel A and voxel B contain observations, merge them.
    std::map<Label, LabelConfidence> total_label_counts;

    for (LabelCount label_count : voxel_A.label_count) {
      if (label_count.label != 0) {
        total_label_counts.emplace(label_count.label,
                                   label_count.label_confidence);
      }
    }

    for (LabelCount label_count : voxel_B->label_count) {
      if (label_count.label != 0) {
        auto it = total_label_counts.find(label_count.label);
        if (it != total_label_counts.end()) {
          total_label_counts[label_count.label] =
              total_label_counts[label_count.label] +
              label_count.label_confidence;
        } else {
          total_label_counts.emplace(label_count.label,
                                     label_count.label_confidence);
        }
      }
    }

    std::vector<std::pair<Label, LabelConfidence>> label_counts;
    std::copy(
        total_label_counts.begin(), total_label_counts.end(),
        std::back_inserter<std::vector<std::pair<Label, LabelConfidence>>>(
            label_counts));

    std::sort(label_counts.begin(), label_counts.end(),
              [](const std::pair<Label, LabelConfidence>& l,
                 const std::pair<Label, LabelConfidence>& r) {
                if (l.second != r.second) return l.second < r.second;
                return l.first < r.first;
              });

    int i = 0;
    while (i < label_count_size && i < label_counts.size()) {
      voxel_B->label_count[i].label = label_counts[i].first;
      voxel_B->label_count[i].label_confidence = label_counts[i].second;

      if (i == 0) {
        voxel_B->label = label_counts[i].first;
        voxel_B->label_confidence = label_counts[i].second;
      }
      ++i;
    }
  }
}

}  // namespace voxblox
