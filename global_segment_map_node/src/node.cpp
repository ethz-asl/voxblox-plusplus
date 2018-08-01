// Copyright 2017 Margarita Grinvald, ASL, ETH Zurich, Switzerland

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>

#include "voxblox_gsm/sliding_window/spatial_sliding_window.h"
#include "voxblox_gsm/sliding_window/time_based_sliding_window.h"

enum class GsmMode { kVanilla, kSpatialSlidingWindow, kTimeBasedSlidingWindow };
typedef std::unique_ptr<voxblox::voxblox_gsm::Controller> ControllerPtr;

ControllerPtr controllerFactory(GsmMode mode, ros::NodeHandle* node_handle) {
  switch (mode) {
    case GsmMode::kVanilla:
      LOG(INFO) << "Starting GSM";
      return ControllerPtr(new voxblox::voxblox_gsm::Controller(node_handle));

    case GsmMode::kSpatialSlidingWindow:
      LOG(INFO) << "Starting spatial sliding window GSM";
      return ControllerPtr(
          new voxblox::voxblox_gsm::SpatialSlidingWindow(node_handle));

    case GsmMode::kTimeBasedSlidingWindow:
      LOG(INFO) << "Starting time based sliding window GSM";
      return ControllerPtr(
          new voxblox::voxblox_gsm::TimeBasedSlidingWindow(node_handle));

    default:
      LOG(FATAL) << "The selected gsm mode was not implemented yet.";
      break;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "gsm_node");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  FLAGS_stderrthreshold = 1;
  ros::NodeHandle node_handle;
  ros::NodeHandle node_handle_private("~");

  int gsm_mode = 0;
  node_handle_private.param<int>("gsm_mode", gsm_mode, gsm_mode);

  ControllerPtr controller =
      controllerFactory(static_cast<GsmMode>(gsm_mode), &node_handle_private);

  ros::Subscriber segment_point_cloud_sub;
  controller->subscribeSegmentPointCloudTopic(&segment_point_cloud_sub);

  ros::Publisher segment_gsm_update_publisher;
  controller->advertiseSegmentGsmUpdateTopic(&segment_gsm_update_publisher);

  ros::Publisher scene_gsm_update_publisher;
  controller->advertiseSceneGsmUpdateTopic(&scene_gsm_update_publisher);

  ros::Publisher segment_mesh_publisher;
  if (controller->publish_segment_mesh_) {
    controller->advertiseSegmentMeshTopic(&segment_mesh_publisher);
  }

  ros::Publisher scene_mesh_publisher;
  if (controller->publish_scene_mesh_) {
    controller->advertiseSceneMeshTopic(&scene_mesh_publisher);
  }

  ros::ServiceServer publish_scene_srv;
  controller->advertisePublishSceneService(&publish_scene_srv);

  ros::ServiceServer validate_merged_object_srv;
  controller->validateMergedObjectService(&validate_merged_object_srv);

  ros::ServiceServer generate_mesh_srv;
  controller->advertiseGenerateMeshService(&generate_mesh_srv);

  ros::ServiceServer extract_segments_srv;
  controller->advertiseExtractSegmentsService(&extract_segments_srv);

  while (ros::ok() && !controller->noNewUpdatesReceived()) {
    ros::spinOnce();
  }

  controller->publishScene();
  constexpr bool kPublishAllSegments = true;
  controller->publishObjects(kPublishAllSegments);

  LOG(INFO) << "Shutting down.";
  return 0;
}
