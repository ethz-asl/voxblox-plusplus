// Copyright (c) 2019, ASL, ETH Zurich, Switzerland
// Licensed under the BSD 3-Clause License (see LICENSE for details)

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

  voxblox::voxblox_gsm::Controller* controller;
  LOG(INFO) << "Starting Voxblox++";
  controller = new voxblox::voxblox_gsm::Controller(&node_handle_private);

  ros::Subscriber segment_point_cloud_sub;
  controller->subscribeSegmentPointCloudTopic(&segment_point_cloud_sub);

  if (controller->publish_scene_mesh_) {
    controller->advertiseSceneMeshTopic();
  }

  ros::Publisher tsdf_slice_publisher;
  ros::Publisher tsdf_publisher;
  ros::Publisher label_tsdf_publisher;

  if (controller->publish_pointclouds_) {
    controller->advertiseTsdfSliceTopic(&tsdf_slice_publisher);
    controller->advertiseTsdfTopic(&tsdf_publisher);
    controller->advertiseLabelTsdfTopic(&label_tsdf_publisher);
  }

  if (controller->compute_and_publish_bbox_) {
    controller->advertiseBboxTopic();
  }

  ros::ServiceServer generate_mesh_srv;
  controller->advertiseGenerateMeshService(&generate_mesh_srv);

  ros::ServiceServer save_segments_as_mesh_srv;
  controller->advertiseSaveSegmentsAsMeshService(&save_segments_as_mesh_srv);

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

  LOG(INFO) << "Shutting down.";
  return 0;
}
