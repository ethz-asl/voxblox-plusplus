// Copyright 2017 Margarita Grinvald, ASL, ETH Zurich, Switzerland

#include "voxblox_gsm/controller.h"

#include <glog/logging.h>
#include <modelify/object_toolbox/object_toolbox.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include "voxblox/io/mesh_ply.h"
#include "voxblox/mesh/mesh_label_integrator.h"
#include <minkindr_conversions/kindr_tf.h>

#include <string>

namespace voxblox_gsm {

Controller::Controller(ros::NodeHandle* node_handle)
    : node_handle_(node_handle) {
  CHECK_NOTNULL(node_handle_);

  voxblox::LabelTsdfMap::Config map_config;
  map_config.voxel_size = 0.01f;
  map_config.voxels_per_side = 8u;
  map_.reset(new voxblox::LabelTsdfMap(map_config));

  voxblox::LabelTsdfIntegrator::Config integrator_config;
  integrator_.reset(
      new voxblox::LabelTsdfIntegrator(integrator_config,
                                       map_->getTsdfLayerPtr(),
                                       map_->getLabelLayerPtr(),
                                       map_->getHighestLabelPtr()));
}

Controller::~Controller() {}

bool Controller::lookupTransformTf(const std::string& from_frame,
                                    const std::string& to_frame,
                                    const ros::Time& timestamp,
                                    voxblox::Transformation* transform) {
  tf::StampedTransform tf_transform;
  ros::Time time_to_lookup = timestamp;

  // If this transform isn't possible at the time, then try to just look up
  // the latest (this is to work with bag files and static transform
  // publisher, etc).
  if (!tf_listener_.canTransform(to_frame, from_frame, time_to_lookup)) {
    time_to_lookup = ros::Time(0);
    ROS_WARN("Using latest TF transform instead of timestamp match.");
  }

  try {
    tf_listener_.lookupTransform(to_frame, from_frame, time_to_lookup,
                                 tf_transform);
  } catch (tf::TransformException& ex) {  // NOLINT
    return false;
  }

  tf::transformTFToKindr(tf_transform, transform);
  return true;
}

void Controller::SegmentPointCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& segment_pc2_msg) {
  voxblox::Transformation T_G_C;

  if (lookupTransformTf(segment_pc2_msg->header.frame_id, "world",
                      segment_pc2_msg->header.stamp, &T_G_C)) {
    modelify::PointCloudType point_cloud;
    pcl::fromROSMsg(*segment_pc2_msg, point_cloud);
    LOG(ERROR) << point_cloud.points.size();

    voxblox::Pointcloud points_C;
    voxblox::Colors colors;
    voxblox::Labels labels_to_integrate;
    points_C.reserve(point_cloud.size());
    colors.reserve(point_cloud.size());
    labels_to_integrate.reserve(point_cloud.size());
    for (size_t i = 0; i < point_cloud.points.size(); ++i) {
      points_C.push_back(voxblox::Point(point_cloud.points[i].x,
                               point_cloud.points[i].y,
                               point_cloud.points[i].z));
      colors.push_back(
          voxblox::Color(point_cloud.points[i].r, point_cloud.points[i].g,
                         point_cloud.points[i].b, point_cloud.points[i].a));
    }

    integrator_->computePointCloudLabel(T_G_C,
                                            points_C,
                                            &labels_to_integrate);

    integrator_->integratePointCloud(T_G_C,
                                         points_C,
                                         colors,
                                           labels_to_integrate);

  }



//  try {
//    listener.waitForTransform("/camera_depth_optical_frame", "/world", ros::Time(0), ros::Duration(10.0) );
//    listener.lookupTransform("/camera_depth_optical_frame", "/world",
//                             ros::Time(0), tf_transform);
//    LOG(ERROR) << "couple";
//    LOG(ERROR) << tf_transform.stamp_;
//    LOG(ERROR) << segment_pc2_msg->header.stamp;
//  }
//  catch (tf::TransformException &ex) {
//    LOG(ERROR) << ex.what();
//  }

//  //while (node_handle_->ok()){
//    try {
//      listener.waitForTransform("/camera_depth_optical_frame", "/world", ros::Time(0), ros::Duration(10.0) );
//      listener.lookupTransform("/camera_depth_optical_frame", "/world",
//                               ros::Time(0), tf_transform);
//      LOG(ERROR) << "couple";
//      LOG(ERROR) << tf_transform.stamp_;
//      LOG(ERROR) << segment_pc2_msg->header.stamp;
//    }
//    catch (tf::TransformException &ex) {
//      LOG(ERROR) << ex.what();
//    }
//
//    voxblox::Labels labels_to_integrate;
//
//    voxblox::Point sensor_origin(0, 0, 0);
//
//    voxblox::Transformation transformation;
//    tf::transformTFToKindr(tf_transform, &transformation);

  //labels_to_integrate.reserve(point_cloud.points.size());

//    voxblox::Pointcloud points_C;
//    voxblox::Colors colors_to_integrate;
//    points_C.reserve(point_cloud.size());
//    colors_to_integrate.reserve(point_cloud.size());
//    labels_to_integrate.reserve(point_cloud.size());
//
//    LOG(ERROR) << point_cloud.points.size();
//    for (size_t i = 0; i < point_cloud.points.size(); ++i) {
//      points_C.push_back(voxblox::Point(point_cloud.points[i].x,
//                               point_cloud.points[i].y,
//                               point_cloud.points[i].z));
//      colors_to_integrate.push_back(
//          voxblox::Color(point_cloud.points[i].r, point_cloud.points[i].g,
//                point_cloud.points[i].b, point_cloud.points[i].a));
//    }
//
//    integrator_->computePointCloudLabel(transformation,
//                                        points_C,
//                                        &labels_to_integrate);
//
//    for (int i = 0; i < labels_to_integrate.size(); i++) {
//      LOG(ERROR) << labels_to_integrate[i];
//    }
//
//    integrator_->integratePointCloud(transformation,
//                                     points_C,
//                                       colors_to_integrate,
//                                       labels_to_integrate);
//
    voxblox::MeshLayer mesh_layer(map_->block_size());
    voxblox::MeshLabelIntegrator::Config mesh_config;
    voxblox::MeshLabelIntegrator mesh_integrator(mesh_config,
                                        map_->getTsdfLayerPtr(),
                                        map_->getLabelLayerPtr(),
                                        &mesh_layer);

    mesh_integrator.generateWholeMesh();

    voxblox::outputMeshLayerAsPly("test.ply", mesh_layer);
// // }
//
//  voxblox::Labels computed_labels;
//  integrator_->computePointCloudLabel(transform,
//                                      frame_to_compute_labels,
//                                      &computed_labels);

//  Object object;
//  object.SetPointCloud(point_cloud);
//  ComputeObjectData(&object);
//
//  ProcessNewObject(&object);
}

void Controller::SubscribeSegmentPointCloudTopic(
    ros::Subscriber* segment_point_cloud_sub) {
  CHECK_NOTNULL(segment_point_cloud_sub);

  std::string segment_point_cloud_topic;
  node_handle_->param<std::string>("segment_point_cloud_topic",
                                   segment_point_cloud_topic,
                                   "/depth_segmentation_node/object_segment");

  *segment_point_cloud_sub =
      node_handle_->subscribe(segment_point_cloud_topic, 1,
                              &Controller::SegmentPointCloudCallback, this);




//  std::string point_cloud2_topic;
//  node_handle_->param<std::string>("point_cloud2_topic",
//                                   point_cloud2_topic,
//                                   "/depth_segmentation_node/object_segment");
//
//  *segment_point_cloud_sub =
//      node_handle_->subscribe(point_cloud2_topic, 1,
//                              &Controller::SegmentPointCloudCallback, this);
//
//
//  std::string tf2_topic;
//  node_handle_->param<std::string>("tf2_topic",
//                                   tf2_topic,
//                                   "/tf");
//
//  *tf_sub =
//      node_handle_->subscribe(tf2_topic, 1,
//                              &Controller::SegmentPointCloudCallback, this);
//


//  message_filters::Subscriber<tf2_msgs::TFMessage> sub2(*node_handle_, "tf2_topic", 1);


//  tf::MessageFilter<sensor_msgs::PointCloud2> * tf_filter_ =
//      new tf::MessageFilter<sensor_msgs::PointCloud2>(sub1, tf_, "/point_cloud2_topic", 10);
//
//  tf_filter_->registerCallback( boost::bind(&PoseDrawer::msgCallback, this, _1) );


//  message_filters::Subscriber<sensor_msgs::PointCloud2> sub1(
//        *node_handle_, "/point_cloud2_topic", 1);
//
//  tf::TransformListener tf_;
//
//  std::string target_frame_;
//
//  tf::MessageFilter<sensor_msgs::PointCloud2> * tf_filter_ =
//       new tf::MessageFilter<sensor_msgs::PointCloud2>(sub1, tf_, "camera_depth_optical_frame", 10);
//  tf_filter_->registerCallback( boost::bind(&Controller::SegmentPointCloudCallback, this, _1) );
//

//
//  message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, tf2_msgs::TFMessage> sync(
//      sub1, sub2, 2);
//
//  sync.registerCallback(boost::bind(&Controller::SegmentPointCloudCallbackk, this, _1, _2));
}
}  // namespace voxblox_gsm
