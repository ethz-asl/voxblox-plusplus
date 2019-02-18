#include "global_segment_map/icp_utils.h"

#include <gflags/gflags.h>

DEFINE_bool(icp_refine_roll_pitch, false,
            "If enabled, ICP will refine not only translation and yaw, but "
            "also roll and pitch.");
DEFINE_int32(
    icp_mini_batch_size, 20,
    "Number of points used in each alignment step. To allow simple threading "
    "the ICP process is split up into a large number of separate alignments "
    "performed on small pointclouds. This parameter dictates how many points "
    "are used in each 'mini batch'. The result are then combined weighting "
    "them by an estimate of the information gained by the alignment.");
DEFINE_double(icp_min_match_ratio, 0.8,
              "Ratio of points that must lie within the truncation distance of "
              "an allocated voxel");
DEFINE_double(icp_subsample_keep_ratio, 0.5,
              "Ratio of points used in the ICP matching");
DEFINE_double(
    icp_inital_translation_weighting, 100,
    "Weighting applied to the translational component of the initial guess. "
    "Very roughly corresponds to the inverse covariance of the initial guess "
    "multiplied by the variance in a measured points accuracy.");
DEFINE_double(icp_inital_rotation_weighting, 100,
              "Weighting applied to the rotational component of the initial "
              "guess. See inital_translation_weighting for further details");

namespace voxblox {
ICP::Config getICPConfigFromGflags() {
  ICP::Config icp_config;

  icp_config.refine_roll_pitch = FLAGS_icp_refine_roll_pitch;
  icp_config.mini_batch_size = FLAGS_icp_mini_batch_size;
  icp_config.min_match_ratio = FLAGS_icp_min_match_ratio;
  icp_config.subsample_keep_ratio = FLAGS_icp_subsample_keep_ratio;
  icp_config.inital_translation_weighting =
      FLAGS_icp_inital_translation_weighting;
  icp_config.inital_rotation_weighting = FLAGS_icp_inital_rotation_weighting;

  return icp_config;
}

}  // namespace voxblox
