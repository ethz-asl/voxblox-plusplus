// Copyright 2017 Margarita Grinvald, ASL, ETH Zurich, Switzerland

#include <glog/logging.h>
#include <gflags/gflags.h>
#include <ros/ros.h>

#include "voxblox_gsm/controller.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "gsm_node");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  LOG(INFO) << "Running GSM...";

  ros::NodeHandle node_handle;
  ros::NodeHandle node_handle_private("~");

  voxblox_gsm::Controller controller(&node_handle_private);

  ros::Subscriber segment_point_cloud_sub;
  controller.subscribeSegmentPointCloudTopic(&segment_point_cloud_sub);

  ros::ServiceServer validate_merged_object_srv;
  controller.validateMergedObjectService(&validate_merged_object_srv);

  ros::Publisher mesh_publisher;
  controller.advertiseMeshTopic(&mesh_publisher);

  ros::Publisher scene_publisher;
  controller.advertiseSceneTopic(&scene_publisher);

  ros::Publisher object_publisher;
  controller.advertiseObjectTopic(&object_publisher);

  ros::Publisher gsm_update_publisher;
  controller.advertiseGsmUpdateTopic(&gsm_update_publisher);

  ros::ServiceServer generate_mesh_srv;
  controller.advertiseGenerateMeshService(&generate_mesh_srv);

  ros::ServiceServer publish_scene_srv;
  controller.advertisePublishSceneService(&publish_scene_srv);

  ros::ServiceServer extract_segments_srv;
  controller.advertiseExtractSegmentsService(&extract_segments_srv);

  constexpr double kNoUpdateTimeout = 20.0;
  while (ros::ok() && !controller.noNewUpdatesReceived(kNoUpdateTimeout)) {
    ros::spinOnce();
  }
  LOG(INFO) << "Shutting down";
  controller.publishScene();
  constexpr bool kPublishAllSegments = true;
  controller.publishObjects(kPublishAllSegments);
  return 0;
}
