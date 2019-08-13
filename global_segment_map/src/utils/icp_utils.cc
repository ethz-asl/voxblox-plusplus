#include "global_segment_map/utils/icp_utils.h"

#include <gflags/gflags.h>

DEFINE_bool(icp_refine_roll_pitch, false,
            "If enabled, ICP will refine not only translation and yaw, but "
            "also roll and pitch.");
DEFINE_int32(
    icp_mini_batch_size, 200,
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

bool refinePointCloudLayerTransform(const Layer<TsdfVoxel>& tsdf_layer,
                                    const Pointcloud& point_cloud,
                                    const Transformation& T_init, ICP* icp_p,
                                    Transformation* T_icp_p,
                                    Transformation* T_offset_p) {
  if (tsdf_layer.getNumberOfAllocatedBlocks() <= 0u) {
    LOG(ERROR) << "TSDF layer is empty, cannot refine pointcloud-to-layer "
                  "alignment.";
    return false;
  }
  timing::Timer icp_timer("ICP");

  // TODO(margaritaG) : account for correction.
  // if (!label_tsdf_config_.keep_track_of_icp_correction) {
  //   T_G_G_icp_.setIdentity();
  // }
  // const size_t num_icp_updates = icp_->runICP(
  //     tsdf_layer, point_cloud, T_G_G_icp_ * T_G_C_init, &T_G_C_icp);

  const size_t num_icp_updates =
      icp_p->runICP(tsdf_layer, point_cloud, T_init, T_icp_p);

  // if (num_icp_updates == 0u || num_icp_updates > 800u) {
  //   LOG(ERROR) << "Resulting num_icp_updates is too high or 0: "
  //              << num_icp_updates << ", using T_G_C_init.";
  //   return T_G_C_init;
  // }

  LOG(ERROR) << "ICP refinement performed " << num_icp_updates
             << " successful update steps.";
  *T_offset_p = *T_icp_p * T_init.inverse();

  if (!icp_p->refiningRollPitch()) {
    // It is already removed internally but small floating point errors can
    // build up if accumulating transforms.
    Transformation::Vector6 vec_T_offset = T_offset_p->log();
    vec_T_offset[3] = 0.0;
    vec_T_offset[4] = 0.0;
    *T_offset_p = Transformation::exp(vec_T_offset);
  }

  // if (!label_tsdf_config_.keep_track_of_icp_correction) {
  //   LOG(ERROR) << "Current ICP refinement offset: T_Gicp_G: " << T_G_G_icp_;
  // } else {
  //   LOG(ERROR) << "ICP refinement for this pointcloud: T_Gicp_G: "
  //              << T_G_G_icp_;
  // }

  icp_timer.Stop();

  return true;
}

}  // namespace voxblox
