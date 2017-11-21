// Copyright 2017 Margarita Grinvald, ASL, ETH Zurich, Switzerland

#include <glog/logging.h>
#include <ros/ros.h>

#include "voxblox_gsm/controller.h"

int main(int argc, char** argv) {
  google::InitGoogleLogging("gsm_node");

  ros::init(argc, argv, "gsm_node");

  ros::NodeHandle node_handle;
  ros::NodeHandle node_handle_private("~");

  voxblox_gsm::Controller controller(&node_handle_private);

  ros::Subscriber segment_point_cloud_sub;
  controller.subscribeSegmentPointCloudTopic(&segment_point_cloud_sub);

  ros::Publisher mesh_publisher;
  controller.advertiseMeshTopic(&mesh_publisher);

  ros::ServiceServer generate_mesh_srv;
  controller.advertiseGenerateMeshService(&generate_mesh_srv);

  ros::ServiceServer extract_segments_srv;
  controller.advertiseExtractSegmentsService(&extract_segments_srv);

  ros::spin();
  return 0;
}
