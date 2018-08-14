// Copyright 2017 Margarita Grinvald, ASL, ETH Zurich, Switzerland

#include <functional>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>

#include "voxblox_gsm/controller.h"

namespace {
std::function<void(int)> shutdown_callback;
void signal_handler(int signum) {
  shutdown_callback(signum);
}
}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "gsm_node");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  ros::NodeHandle node_handle;
  ros::NodeHandle node_handle_private("~");

  voxblox::voxblox_gsm::Controller controller(&node_handle_private);

  shutdown_callback = [&](int signum) {
    LOG(INFO) << "Received shutdown signal: " << signum;

    LOG(INFO) << "Publishing Scene...";
    controller.publishScene();
    LOG(INFO) << "done.";

    LOG(INFO) << "Publishing Objects...";
    constexpr bool kPublishAllSegments = true;
    controller.publishObjects(kPublishAllSegments);
    LOG(INFO) << "done.";

    ros::spinOnce();

    ros::shutdown();
  };
  signal(SIGINT, signal_handler);

  ros::Subscriber segment_point_cloud_sub;
  controller.subscribeSegmentPointCloudTopic(&segment_point_cloud_sub);

  ros::Publisher segment_gsm_update_publisher;
  controller.advertiseSegmentGsmUpdateTopic(&segment_gsm_update_publisher);

  ros::Publisher scene_gsm_update_publisher;
  controller.advertiseSceneGsmUpdateTopic(&scene_gsm_update_publisher);

  ros::Publisher segment_mesh_publisher;
  if (controller.publish_segment_mesh_) {
    controller.advertiseSegmentMeshTopic(&segment_mesh_publisher);
  }

  ros::Publisher scene_mesh_publisher;
  if (controller.publish_scene_mesh_) {
    controller.advertiseSceneMeshTopic(&scene_mesh_publisher);
  }

  ros::ServiceServer publish_scene_srv;
  controller.advertisePublishSceneService(&publish_scene_srv);

  ros::ServiceServer validate_merged_object_srv;
  controller.validateMergedObjectService(&validate_merged_object_srv);

  ros::ServiceServer generate_mesh_srv;
  controller.advertiseGenerateMeshService(&generate_mesh_srv);

  ros::ServiceServer extract_segments_srv;
  controller.advertiseExtractSegmentsService(&extract_segments_srv);

  while (ros::ok() && !controller.noNewUpdatesReceived()) {
    ros::spinOnce();
  }

  // If we exited the ros loop due to time out, call the proper shutdown
  // procedure.
  if (ros::ok() && controller.noNewUpdatesReceived()) {
    shutdown_callback(0);
  }

  LOG(INFO) << "Shutting down.";
  return 0;
}
