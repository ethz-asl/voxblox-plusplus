// Copyright 2017 Margarita Grinvald, ASL, ETH Zurich, Switzerland

#include <glog/logging.h>
#include <ros/ros.h>

#include "voxblox_gsm/controller.h"

int main(int argc, char** argv) {
  google::InitGoogleLogging("voxblox_gsm");

  ros::init(argc, argv, "voxblox_gsm_node");

  ros::NodeHandle node_handle;
  voxblox_gsm::Controller controller(&node_handle);

  ros::Subscriber segment_point_cloud_sub;
  controller.SubscribeSegmentPointCloudTopic(&segment_point_cloud_sub);

  ros::spin();

  return 0;
}
