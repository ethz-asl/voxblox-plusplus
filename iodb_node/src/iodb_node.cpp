// Copyright (c) 2019, ASL, ETH Zurich, Switzerland
// Licensed under the BSD 3-Clause License (see LICENSE for details)

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>

#include <global_segment_map_node/controller.h>
#include <iodb_node/iodb_controller.h>
#include <iodb_node/sliding_window_controller.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "iodb_node");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  FLAGS_stderrthreshold = 1;
  ros::NodeHandle node_handle;
  ros::NodeHandle node_handle_private("~");

  bool sliding_window = false;
  node_handle_private.param<bool>("sliding_window/is_on", sliding_window,
                                  sliding_window);

  voxblox::voxblox_gsm::IodbController* controller;

  if (sliding_window) {
    LOG(INFO) << "Starting sliding window GSM";
    controller =
        new voxblox::voxblox_gsm::SlidingWindowController(&node_handle_private);
  } else {
    LOG(INFO) << "Starting IODB";
    controller = new voxblox::voxblox_gsm::IodbController(&node_handle_private);
  }

  ros::Subscriber feature_sub;
  controller->subscribeFeatureTopic(&feature_sub);

  if (controller->publish_gsm_updates_) {
    controller->advertiseSegmentGsmUpdateTopic();

    controller->advertiseSceneGsmUpdateTopic();
  }

  if (controller->publish_segment_mesh_) {
    controller->advertiseSegmentMeshTopic();
  }

  if (controller->publish_feature_blocks_marker_) {
    controller->advertiseFeatureBlockTopic();
  }

  ros::ServiceServer publish_scene_srv;
  controller->advertisePublishSceneService(&publish_scene_srv);

  ros::ServiceServer validate_merged_object_srv;
  controller->validateMergedObjectService(&validate_merged_object_srv);

  ros::Subscriber segment_point_cloud_sub;
  controller->subscribeSegmentPointCloudTopic(&segment_point_cloud_sub);

  if (controller->publish_segment_mesh_) {
    controller->advertiseSegmentMeshTopic();
  }

  if (controller->publish_scene_mesh_) {
    controller->advertiseSceneMeshTopic();
  }

  if (controller->compute_and_publish_bbox_) {
    controller->advertiseBboxTopic();
  }

  ros::ServiceServer generate_mesh_srv;
  controller->advertiseGenerateMeshService(&generate_mesh_srv);

  ros::ServiceServer save_segments_as_mesh_srv;
  controller->advertiseSaveSegmentsAsMeshService(&save_segments_as_mesh_srv);

  ros::ServiceServer extract_instances_srv;
  if (controller->enable_semantic_instance_segmentation_) {
    controller->advertiseExtractInstancesService(&extract_instances_srv);
  }

  // Spinner that uses a number of threads equal to the number of cores.
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();

  if (controller->publish_scene_mesh_) {
    controller->publishScene();
    constexpr bool kPublishAllSegments = true;
    controller->publishObjects(kPublishAllSegments);
  }

  LOG(INFO) << "Shutting down.";
  return 0;
}
