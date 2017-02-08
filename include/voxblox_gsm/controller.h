// Copyright 2017 Margarita Grinvald, ASL, ETH Zurich, Switzerland

#ifndef VOXBLOX_GSM_INCLUDE_VOXBLOX_GSM_CONTROLLER_H_
#define VOXBLOX_GSM_INCLUDE_VOXBLOX_GSM_CONTROLLER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <voxblox/core/labeltsdf_map.h>
#include <voxblox/integrator/labeltsdf_integrator.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <tf2_msgs/TFMessage.h>

namespace voxblox_gsm {

class Controller {
 public:
  Controller(ros::NodeHandle* node_handle);

  ~Controller();

  void SubscribeSegmentPointCloudTopic(
      ros::Subscriber* segment_point_cloud_sub);

 private:
  void SegmentPointCloudCallback(
      const sensor_msgs::PointCloud2::ConstPtr& segment_pc2_msg);

  void SegmentPointCloudCallbackk(
      const sensor_msgs::PointCloud2::ConstPtr& segment_pc2_msg,
      const tf2_msgs::TFMessageConstPtr& tf_msg);

  bool lookupTransformTf(const std::string& from_frame,
                                      const std::string& to_frame,
                                      const ros::Time& timestamp,
                                      voxblox::Transformation* transform);

  ros::NodeHandle* node_handle_;

  tf::TransformListener tf_listener_;

  std::shared_ptr<voxblox::LabelTsdfMap> map_;
  std::shared_ptr<voxblox::LabelTsdfIntegrator> integrator_;
};

}  // namespace voxblox_gsm

#endif  // VOXBLOX_GSM_INCLUDE_VOXBLOX_GSM_CONTROLLER_H_
