// Copyright 2017 Margarita Grinvald, ASL, ETH Zurich, Switzerland

#include <dynamic_reconfigure/server.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>

#include "gsm_node/InteractiveSliderConfig.h"
#include "voxblox_gsm/controller.h"
#include "voxblox_gsm/sliding_window_controller.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "gsm_node");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  FLAGS_stderrthreshold = 1;
  ros::NodeHandle node_handle;
  ros::NodeHandle node_handle_private("~");

  bool sliding_window = false;
  node_handle_private.param<bool>("sliding_window/is_on", sliding_window,
                                  sliding_window);

  voxblox::voxblox_gsm::Controller* controller;

  if (sliding_window) {
    LOG(INFO) << "Starting sliding window GSM";
    controller =
        new voxblox::voxblox_gsm::SlidingWindowController(&node_handle_private);
  } else {
    LOG(INFO) << "Starting GSM";
    controller = new voxblox::voxblox_gsm::Controller(&node_handle_private);
  }

  ros::Subscriber segment_point_cloud_sub;
  controller->subscribeSegmentPointCloudTopic(&segment_point_cloud_sub);

  ros::Publisher segment_gsm_update_publisher;
  ros::Publisher scene_gsm_update_publisher;

  if (controller->publish_gsm_updates_) {
    controller->advertiseSegmentGsmUpdateTopic(&segment_gsm_update_publisher);

    controller->advertiseSceneGsmUpdateTopic(&scene_gsm_update_publisher);
  }

  ros::Publisher segment_mesh_publisher;
  if (controller->publish_segment_mesh_) {
    controller->advertiseSegmentMeshTopic(&segment_mesh_publisher);
  }

  ros::Publisher scene_mesh_publisher;
  if (controller->publish_scene_mesh_) {
    controller->advertiseSceneMeshTopic(&scene_mesh_publisher);
  }

  ros::Publisher tsdf_slice_publisher;
  ros::Publisher tsdf_publisher;
  ros::Publisher label_tsdf_publisher;

  if (controller->publish_pointclouds_) {
    controller->advertiseTsdfSliceTopic(&tsdf_slice_publisher);
    controller->advertiseTsdfTopic(&tsdf_publisher);
    controller->advertiseLabelTsdfTopic(&label_tsdf_publisher);
  }

  ros::Publisher bbox_pub;
  if (controller->compute_and_publish_bbox_) {
    controller->advertiseBboxTopic(&bbox_pub);
  }

  ros::ServiceServer publish_scene_srv;
  controller->advertisePublishSceneService(&publish_scene_srv);

  ros::ServiceServer validate_merged_object_srv;
  controller->validateMergedObjectService(&validate_merged_object_srv);

  ros::ServiceServer generate_mesh_srv;
  controller->advertiseGenerateMeshService(&generate_mesh_srv);

  ros::ServiceServer extract_segments_srv;
  controller->advertiseExtractSegmentsService(&extract_segments_srv);

  dynamic_reconfigure::Server<gsm_node::InteractiveSliderConfig>
      reconfigure_server;
  dynamic_reconfigure::Server<gsm_node::InteractiveSliderConfig>::CallbackType
      dynamic_reconfigure_function;

  dynamic_reconfigure_function =
      std::bind(&voxblox::voxblox_gsm::Controller::dynamicReconfigureCallback,
                controller, std::placeholders::_1, std::placeholders::_2);
  reconfigure_server.setCallback(dynamic_reconfigure_function);

  ros::ServiceServer extract_instances_srv;
  if (controller->enable_semantic_instance_segmentation_) {
    controller->advertiseExtractInstancesService(&extract_instances_srv);
  }

  // Spinner that uses a number of threads equal to the number of cores.
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();

  if (controller->publish_gsm_updates_) {
    controller->publishScene();
    constexpr bool kPublishAllSegments = true;
    controller->publishObjects(kPublishAllSegments);
  }

  LOG(INFO) << "Shutting down.";
  return 0;
}
