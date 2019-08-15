// Copyright (c) 2019, ASL, ETH Zurich, Switzerland
// Licensed under the BSD 3-Clause License (see LICENSE for details)

#include "iodb_node/iodb_controller.h"

#include <global_segment_map/meshing/label_tsdf_mesh_integrator.h>
#include <global_segment_map_node/conversions.h>
#include <global_segment_map_node/feature_ros_tools.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <global_segment_map/meshing/label_tsdf_mesh_integrator.h>
#include <iodb_node/conversions.h>
#include <iodb_node/feature_ros_tools.h>
#include <voxblox/integrator/merge_integration.h>
#include <voxblox/utils/layer_utils.h>
#include <voxblox_ros/mesh_vis.h>

#include "iodb_node/conversions.h"

namespace voxblox {
namespace voxblox_gsm {

IodbController::IodbController(ros::NodeHandle* node_handle)
    : Controller(node_handle),
      publish_gsm_updates_(false),
      no_update_timeout_(0.0),
      min_number_of_allocated_blocks_to_publish_(10),
      received_first_feature_(false),
      publish_feature_blocks_marker_(false) {
  node_handle_private_->param<int>(
      "object_database/max_segment_age",
      label_tsdf_integrator_config_.max_segment_age,
      label_tsdf_integrator_config_.max_segment_age);

  node_handle_private_->param<bool>("object_database/publish_gsm_updates",
                                    publish_gsm_updates_, publish_gsm_updates_);

  node_handle_private_->param<bool>("meshing/publish_segment_mesh",
                                    publish_segment_mesh_,
                                    publish_segment_mesh_);

  node_handle_private_->param<double>("object_database/no_update_timeout",
                                      no_update_timeout_, no_update_timeout_);

  const double kBlockSize =
      map_config_.voxel_size * map_config_.voxels_per_side;
  feature_layer_.reset(new FeatureLayer<Feature3D>(kBlockSize));

  // Determine feature integrator parameters.
  // TODO(ntonci): Make rosparam if there are any.
  FeatureIntegrator::Config feature_integrator_config;
  feature_integrator_.reset(
      new FeatureIntegrator(feature_integrator_config, feature_layer_.get()));

  node_handle_private_->param<bool>("publish_feature_blocks_marker",
                                    publish_feature_blocks_marker_,
                                    publish_feature_blocks_marker_);

  node_handle_private_->param<int>(
      "object_database/min_number_of_allocated_blocks_to_publish",
      min_number_of_allocated_blocks_to_publish_,
      min_number_of_allocated_blocks_to_publish_);
}

// TODO(ntonci): Move all sub, pub, etc., and helpers to inline header.
void IodbController::subscribeFeatureTopic(ros::Subscriber* feature_sub) {
  CHECK_NOTNULL(feature_sub);
  std::string feature_topic = "/rgbd_feature_node/features";
  node_handle_private_->param<std::string>("feature_topic", feature_topic,
                                           feature_topic);

  // Large queue size to give slack to the pipeline and not lose any messages.
  constexpr int kFeatureQueueSize = 2000;
  *feature_sub = node_handle_private_->subscribe(
      feature_topic, kFeatureQueueSize, &IodbController::featureCallback, this);
}

void IodbController::advertiseSegmentMeshTopic() {
  segment_mesh_pub_ =
      new ros::Publisher(node_handle_private_->advertise<voxblox_msgs::Mesh>(
          "segment_mesh", 1, true));
}

void IodbController::advertiseFeatureBlockTopic() {
  std::string feature_block_topic = "feature_block_array";
  node_handle_private_->param<std::string>(
      "feature_block_topic", feature_block_topic, feature_block_topic);

  constexpr int kFeatureBlockQueueSize = 2000;
  feature_block_pub_ = new ros::Publisher(
      node_handle_private_->advertise<visualization_msgs::MarkerArray>(
          feature_block_topic, kFeatureBlockQueueSize, true));
}

void IodbController::advertiseSegmentGsmUpdateTopic() {
  std::string segment_gsm_update_topic = "gsm_update";
  node_handle_private_->param<std::string>(
      "object_database/segment_gsm_update_topic", segment_gsm_update_topic,
      segment_gsm_update_topic);
  // TODO(ff): Reduce this value, once we know some reasonable limit.
  constexpr int kGsmUpdateQueueSize = 2000;
  segment_gsm_update_pub_ = new ros::Publisher(
      node_handle_private_->advertise<modelify_msgs::GsmUpdate>(
          segment_gsm_update_topic, kGsmUpdateQueueSize, true));
}

void IodbController::advertiseSceneGsmUpdateTopic() {
  std::string scene_gsm_update_topic = "scene";
  node_handle_private_->param<std::string>(
      "object_database/scene_gsm_update_topic", scene_gsm_update_topic,
      scene_gsm_update_topic);
  constexpr int kGsmSceneQueueSize = 1;

  scene_gsm_update_pub_ = new ros::Publisher(
      node_handle_private_->advertise<modelify_msgs::GsmUpdate>(
          scene_gsm_update_topic, kGsmSceneQueueSize, true));
}

void IodbController::advertisePublishSceneService(
    ros::ServiceServer* publish_scene_srv) {
  CHECK_NOTNULL(publish_scene_srv);
  static const std::string kAdvertisePublishSceneServiceName = "publish_scene";
  *publish_scene_srv = node_handle_private_->advertiseService(
      kAdvertisePublishSceneServiceName, &IodbController::publishSceneCallback,
      this);
}

void IodbController::validateMergedObjectService(
    ros::ServiceServer* validate_merged_object_srv) {
  CHECK_NOTNULL(validate_merged_object_srv);
  std::string validate_merged_object_topic = "validate_merged_object";
  node_handle_private_->param<std::string>(
      "object_database/validate_merged_object", validate_merged_object_topic,
      validate_merged_object_topic);

  *validate_merged_object_srv = node_handle_private_->advertiseService(
      validate_merged_object_topic,
      &IodbController::validateMergedObjectCallback, this);
}

void IodbController::extractSegmentLayers(
    const std::vector<Label>& labels,
    std::unordered_map<Label, LayerTuple>* label_layers_map,
    bool labels_list_is_complete) {
  CHECK(label_layers_map);

  // Build map from labels to tsdf and label layers. Each will contain the
  // segment of the corresponding layer.
  Layer<TsdfVoxel> tsdf_layer_empty(map_config_.voxel_size,
                                    map_config_.voxels_per_side);
  Layer<LabelVoxel> label_layer_empty(map_config_.voxel_size,
                                      map_config_.voxels_per_side);
  FeatureLayer<Feature3D> feature_layer_empty(
      map_config_.voxel_size * map_config_.voxels_per_side,
      feature_layer_->getDescriptorSize());

  for (const Label& label : labels) {
    label_layers_map->emplace(
        label, std::make_tuple(tsdf_layer_empty, label_layer_empty,
                               feature_layer_empty));
  }

  BlockIndexList all_label_blocks;
  map_->getTsdfLayerPtr()->getAllAllocatedBlocks(&all_label_blocks);

  for (const BlockIndex& block_index : all_label_blocks) {
    Block<TsdfVoxel>::Ptr global_tsdf_block =
        map_->getTsdfLayerPtr()->getBlockPtrByIndex(block_index);
    Block<LabelVoxel>::Ptr global_label_block =
        map_->getLabelLayerPtr()->getBlockPtrByIndex(block_index);
    FeatureBlock<Feature3D>::Ptr global_feature_block =
        feature_layer_->getBlockPtrByIndex(block_index);

    const size_t vps = global_label_block->voxels_per_side();
    for (size_t i = 0; i < vps * vps * vps; ++i) {
      const LabelVoxel& global_label_voxel =
          global_label_block->getVoxelByLinearIndex(i);

      if (global_label_voxel.label == 0u) {
        continue;
      }

      auto it = label_layers_map->find(global_label_voxel.label);
      if (it == label_layers_map->end()) {
        if (labels_list_is_complete) {
          LOG(FATAL) << "At least one voxel in the GSM is assigned to label "
                     << global_label_voxel.label
                     << " which is not in the given "
                        "list of labels to retrieve.";
        }
        continue;
      }

      Layer<TsdfVoxel>& tsdf_layer =
          std::get<LayerAccessor::kTsdfLayer>(it->second);
      Layer<LabelVoxel>& label_layer =
          std::get<LayerAccessor::kLabelLayer>(it->second);
      FeatureLayer<Feature3D>& feature_layer =
          std::get<LayerAccessor::kFeatureLayer>(it->second);

      Block<TsdfVoxel>::Ptr tsdf_block =
          tsdf_layer.allocateBlockPtrByIndex(block_index);
      Block<LabelVoxel>::Ptr label_block =
          label_layer.allocateBlockPtrByIndex(block_index);
      FeatureBlock<Feature3D>::Ptr feature_block =
          feature_layer.allocateBlockPtrByIndex(block_index);

      CHECK(tsdf_block);
      CHECK(label_block);
      CHECK(feature_block);

      TsdfVoxel& tsdf_voxel = tsdf_block->getVoxelByLinearIndex(i);
      LabelVoxel& label_voxel = label_block->getVoxelByLinearIndex(i);

      const TsdfVoxel& global_tsdf_voxel =
          global_tsdf_block->getVoxelByLinearIndex(i);

      tsdf_voxel = global_tsdf_voxel;
      label_voxel = global_label_voxel;

      if (global_feature_block != nullptr) {
        feature_block->getFeatures() = global_feature_block->getFeatures();
      }
    }
  }
}

void IodbController::getLabelsToPublish(const bool get_all,
                                        std::vector<Label>* labels) {
  CHECK_NOTNULL(labels);
  if (get_all) {
    *labels = map_->getLabelList();
    ROS_INFO("Publishing all segments");
  } else {
    *labels = segment_labels_to_publish_;
  }
}

// TODO(ff): Create this somewhere:
// void serializeGsmAsMsg(const map&, const label&, const parent_labels&,
// msg*);

bool IodbController::publishObjects(const bool publish_all) {
  CHECK_NOTNULL(segment_gsm_update_pub_);
  bool published_segment_label = false;
  std::vector<Label> labels_to_publish;
  std::unordered_map<Label, LayerTuple> label_to_layers;
  ros::Time start;
  {
    std::lock_guard<std::mutex> label_tsdf_layers_lock(
        label_tsdf_layers_mutex_);
    getLabelsToPublish(publish_all, &labels_to_publish);

    start = ros::Time::now();
    extractSegmentLayers(labels_to_publish, &label_to_layers, publish_all);
  }
  ros::Time stop = ros::Time::now();
  ros::Duration duration = stop - start;
  ROS_INFO_STREAM("Extracting segment layers took " << duration.toSec() << "s");

  for (const Label& label : labels_to_publish) {
    auto it = label_to_layers.find(label);
    CHECK(it != label_to_layers.end())
        << "Layers for " << label << " could not be extracted.";

    Layer<TsdfVoxel>& tsdf_layer =
        std::get<LayerAccessor::kTsdfLayer>(it->second);
    Layer<LabelVoxel>& label_layer =
        std::get<LayerAccessor::kLabelLayer>(it->second);
    FeatureLayer<Feature3D>& feature_layer =
        std::get<LayerAccessor::kFeatureLayer>(it->second);

    if (tsdf_layer.getNumberOfAllocatedBlocks() <
        min_number_of_allocated_blocks_to_publish_) {
      continue;
    }

    // Convert to origin and extract translation.
    Point origin_shifted_tsdf_layer_W;
    utils::centerBlocksOfLayer<TsdfVoxel>(&tsdf_layer,
                                          &origin_shifted_tsdf_layer_W);
    // TODO(ff): If this is time consuming we can omit this step.
    Point origin_shifted_label_layer_W;
    utils::centerBlocksOfLayer<LabelVoxel>(&label_layer,
                                           &origin_shifted_label_layer_W);
    CHECK_EQ(origin_shifted_tsdf_layer_W, origin_shifted_label_layer_W);

    // TODO(ntonci): Check that this is centered the same way as tsdf.
    Point origin_shifted_feature_layer_W;
    centerBlocksOfFeatureLayer<Feature3D>(&feature_layer,
                                          &origin_shifted_feature_layer_W);
    CHECK_EQ(origin_shifted_tsdf_layer_W, origin_shifted_feature_layer_W);

    // Extract surfel cloud from layer.
    MeshIntegratorConfig mesh_config;
    node_handle_private_->param<float>(
        "meshing/min_weight", mesh_config.min_weight, mesh_config.min_weight);
    pcl::PointCloud<pcl::PointSurfel>::Ptr surfel_cloud(
        new pcl::PointCloud<pcl::PointSurfel>());
    convertVoxelGridToPointCloud(tsdf_layer, mesh_config, surfel_cloud.get());

    if (surfel_cloud->empty()) {
      LOG(WARNING) << tsdf_layer.getNumberOfAllocatedBlocks()
                   << " blocks didn't produce a surface.";
      LOG(WARNING) << "Labelled segment does not contain enough data to "
                      "extract a surface -> skipping!";
      continue;
    }

    modelify_msgs::GsmUpdate gsm_update_msg;
    constexpr bool kSerializeOnlyUpdated = false;
    gsm_update_msg.header.stamp = last_segment_msg_timestamp_;
    gsm_update_msg.header.frame_id = world_frame_;
    gsm_update_msg.is_scene = false;
    serializeLayerAsMsg<TsdfVoxel>(tsdf_layer, kSerializeOnlyUpdated,
                                   &gsm_update_msg.object.tsdf_layer);
    // TODO(ff): Make sure this works also, there is no LabelVoxel in voxblox
    // yet, hence it doesn't work.
    serializeLayerAsMsg<LabelVoxel>(label_layer, kSerializeOnlyUpdated,
                                    &gsm_update_msg.object.label_layer);
    // TODO(ntonci): Check action and maybe change convention to be in the same
    // style as above.
    feature_layer.serializeLayerAsMsg(kSerializeOnlyUpdated,
                                      DeserializeAction::kUpdate,
                                      &gsm_update_msg.object.feature_layer);
    ROS_INFO_STREAM("Extracted segment with "
                    << surfel_cloud->points.size() << " points and "
                    << feature_layer.getNumberOfFeatures() << " features.");

    gsm_update_msg.object.label = label;
    gsm_update_msg.object.semantic_label =
        map_->getSemanticInstanceLabelFusionPtr()->getSemanticLabel(label);
    gsm_update_msg.old_labels.clear();
    geometry_msgs::Transform transform;
    transform.translation.x = origin_shifted_tsdf_layer_W[0];
    transform.translation.y = origin_shifted_tsdf_layer_W[1];
    transform.translation.z = origin_shifted_tsdf_layer_W[2];
    transform.rotation.w = 1.;
    transform.rotation.x = 0.;
    transform.rotation.y = 0.;
    transform.rotation.z = 0.;
    gsm_update_msg.object.transforms.clear();
    gsm_update_msg.object.transforms.push_back(transform);
    pcl::toROSMsg(*surfel_cloud, gsm_update_msg.object.surfel_cloud);
    gsm_update_msg.object.surfel_cloud.header = gsm_update_msg.header;
    constexpr size_t kNoGTLabel = 0u;
    gsm_update_msg.object.ground_truth_labels.clear();
    gsm_update_msg.object.ground_truth_labels.push_back(kNoGTLabel);

    if (all_published_segments_.find(label) != all_published_segments_.end()) {
      // Segment previously published, sending update message.
      gsm_update_msg.old_labels.push_back(label);
    } else {
      // Segment never previously published, sending first type of message.
    }
    auto merged_label_it = merges_to_publish_.find(label);
    if (merged_label_it != merges_to_publish_.end()) {
      for (Label merged_label : merged_label_it->second) {
        if (all_published_segments_.find(merged_label) !=
            all_published_segments_.end()) {
          gsm_update_msg.old_labels.push_back(merged_label);
        }
      }
      merges_to_publish_.erase(merged_label_it);
    }

    if (compute_and_publish_bbox_) {
      CHECK_NOTNULL(bbox_pub_);

      Eigen::Vector3f bbox_translation;
      Eigen::Quaternionf bbox_quaternion;
      Eigen::Vector3f bbox_size;
      computeAlignedBoundingBox(surfel_cloud, &bbox_translation,
                                &bbox_quaternion, &bbox_size);

      gsm_update_msg.object.bbox.pose.position.x =
          origin_shifted_tsdf_layer_W[0] + bbox_translation(0);
      gsm_update_msg.object.bbox.pose.position.y =
          origin_shifted_tsdf_layer_W[1] + bbox_translation(1);
      gsm_update_msg.object.bbox.pose.position.z =
          origin_shifted_tsdf_layer_W[2] + bbox_translation(2);
      gsm_update_msg.object.bbox.pose.orientation.x = bbox_quaternion.x();
      gsm_update_msg.object.bbox.pose.orientation.y = bbox_quaternion.y();
      gsm_update_msg.object.bbox.pose.orientation.z = bbox_quaternion.z();
      gsm_update_msg.object.bbox.pose.orientation.w = bbox_quaternion.w();
      gsm_update_msg.object.bbox.dimensions.x = bbox_size(0);
      gsm_update_msg.object.bbox.dimensions.y = bbox_size(1);
      gsm_update_msg.object.bbox.dimensions.z = bbox_size(2);

      visualization_msgs::Marker marker;
      marker.header.frame_id = world_frame_;
      marker.header.stamp = ros::Time();
      marker.id = gsm_update_msg.object.label;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x =
          origin_shifted_tsdf_layer_W[0] + bbox_translation(0);
      marker.pose.position.y =
          origin_shifted_tsdf_layer_W[1] + bbox_translation(1);
      marker.pose.position.z =
          origin_shifted_tsdf_layer_W[2] + bbox_translation(2);
      marker.pose.orientation = gsm_update_msg.object.bbox.pose.orientation;
      marker.scale = gsm_update_msg.object.bbox.dimensions;
      marker.color.a = 0.3;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.lifetime = ros::Duration();

      bbox_pub_->publish(marker);

      geometry_msgs::TransformStamped bbox_tf;
      bbox_tf.header = gsm_update_msg.header;
      bbox_tf.child_frame_id = std::to_string(gsm_update_msg.object.label);
      bbox_tf.transform.translation.x = marker.pose.position.x;
      bbox_tf.transform.translation.y = marker.pose.position.y;
      bbox_tf.transform.translation.z = marker.pose.position.z;
      bbox_tf.transform.rotation = marker.pose.orientation;

      tf_broadcaster_.sendTransform(bbox_tf);
    }

    publishGsmUpdate(*segment_gsm_update_pub_, &gsm_update_msg);

    if (publish_segment_mesh_) {
      // TODO(ntonci): Why not call generateMesh?

      // Generate mesh for visualization purposes.
      std::shared_ptr<MeshLayer> mesh_layer;
      mesh_layer.reset(new MeshLayer(tsdf_layer.block_size()));
      // mesh_layer.reset(new MeshLayer(map_->block_size()));
      MeshLabelIntegrator::LabelTsdfConfig label_tsdf_mesh_config;
      label_tsdf_mesh_config.color_scheme =
          MeshLabelIntegrator::ColorScheme::kColor;
      MeshLabelIntegrator mesh_integrator(mesh_config_, label_tsdf_mesh_config,
                                          tsdf_layer, label_layer,
                                          mesh_layer.get());
      constexpr bool only_mesh_updated_blocks = false;
      constexpr bool clear_updated_flag = false;
      {
        std::lock_guard<std::mutex> label_tsdf_layers_lock(
            label_tsdf_layers_mutex_);
        mesh_integrator.generateMesh(only_mesh_updated_blocks,
                                     clear_updated_flag);

        voxblox_msgs::Mesh segment_mesh_msg;
        generateVoxbloxMeshMsg(mesh_layer, ColorMode::kColor,
                               &segment_mesh_msg);
        segment_mesh_msg.header.frame_id = world_frame_;
        segment_mesh_pub_->publish(segment_mesh_msg);
      }
    }
    all_published_segments_.insert(label);
    published_segment_label = true;
  }
  segment_labels_to_publish_.clear();
  return published_segment_label;
}

void IodbController::publishScene() {
  CHECK_NOTNULL(scene_gsm_update_pub_);
  modelify_msgs::GsmUpdate gsm_update_msg;

  // TODO(ntonci): Create a method that generalizes this part such that you dont
  // need to do it twice, for objects and scene.
  gsm_update_msg.header.stamp = last_segment_msg_timestamp_;
  gsm_update_msg.header.frame_id = world_frame_;

  constexpr bool kSerializeOnlyUpdated = false;
  pcl::PointCloud<pcl::PointSurfel>::Ptr surfel_cloud(
      new pcl::PointCloud<pcl::PointSurfel>());
  {
    std::lock_guard<std::mutex> label_tsdf_layers_lock(
        label_tsdf_layers_mutex_);
    serializeLayerAsMsg<TsdfVoxel>(map_->getTsdfLayer(), kSerializeOnlyUpdated,
                                   &gsm_update_msg.object.tsdf_layer);
    // TODO(ff): Make sure this works also, there is no LabelVoxel in voxblox
    // yet, hence it doesn't work.
    // TODO(ntonci): This seems to work?
    serializeLayerAsMsg<LabelVoxel>(map_->getLabelLayer(),
                                    kSerializeOnlyUpdated,
                                    &gsm_update_msg.object.label_layer);

    // TODO(ntonci): Also publish feature layer for scene.

    // TODO(ntonci): This is done twice, rather do it in the beginning with all
    // the other params.
    MeshIntegratorConfig mesh_config;
    node_handle_private_->param<float>("mesh_config/min_weight",
                                       mesh_config.min_weight,
                                       mesh_config.min_weight);

    convertVoxelGridToPointCloud(map_->getTsdfLayer(), mesh_config,
                                 surfel_cloud.get());
  }
  pcl::toROSMsg(*surfel_cloud, gsm_update_msg.object.surfel_cloud);
  gsm_update_msg.object.surfel_cloud.header = gsm_update_msg.header;

  gsm_update_msg.object.label = 0u;
  gsm_update_msg.old_labels.clear();
  gsm_update_msg.is_scene = true;
  geometry_msgs::Transform transform;
  transform.translation.x = 0.0;
  transform.translation.y = 0.0;
  transform.translation.z = 0.0;
  transform.rotation.w = 1.0;
  transform.rotation.x = 0.0;
  transform.rotation.y = 0.0;
  transform.rotation.z = 0.0;
  gsm_update_msg.object.transforms.clear();
  gsm_update_msg.object.transforms.push_back(transform);
  publishGsmUpdate(*scene_gsm_update_pub_, &gsm_update_msg);
}

bool IodbController::noNewUpdatesReceived() const {
  if (received_first_message_ && no_update_timeout_ != 0.0) {
    return (ros::Time::now() - last_update_received_).toSec() >
           no_update_timeout_;
  }
  return false;
}

void IodbController::segmentPointCloudCallback(
    const sensor_msgs::PointCloud2::Ptr& segment_point_cloud_msg) {
  // Message timestamps are used to detect when all
  // segment messages from a certain frame have arrived.
  // Since segments from the same frame all have the same timestamp,
  // the start of a new frame is detected when the message timestamp changes.
  // TODO(grinvalm): need additional check for the last frame to be
  // integrated.
  if (received_first_message_ &&
      last_segment_msg_timestamp_ != segment_point_cloud_msg->header.stamp) {
    if (segments_to_integrate_.size() > 0u) {
      integrateFrame(segment_point_cloud_msg->header.stamp);
      if (publish_gsm_updates_ && publishObjects()) {
        publishScene();
      }
    } else {
      ROS_INFO("No segments to integrate.");
    }
  }

  received_first_message_ = true;
  last_update_received_ = ros::Time::now();
  last_segment_msg_timestamp_ = segment_point_cloud_msg->header.stamp;

  processSegment(segment_point_cloud_msg);
}

void IodbController::featureCallback(
    const modelify_msgs::Features& features_msg) {
  size_t descriptor_size;
  std::string camera_frame;
  ros::Time timestamp;
  std::vector<Feature3D> features_C;

  ros::WallTime start = ros::WallTime::now();

  fromFeaturesMsgToFeature3D(features_msg, &descriptor_size, &camera_frame,
                             &timestamp, &features_C);

  if (!received_first_feature_ && features_C.size() > 0) {
    received_first_feature_ = true;
    feature_layer_->setDescriptorSize(descriptor_size);
  }

  Transformation T_G_C;
  if (lookupTransform(camera_frame, world_frame_, timestamp, &T_G_C)) {
    feature_integrator_->integrateFeatures(T_G_C, features_C);
  } else {
    ROS_WARN_STREAM("Could not find the transform to "
                    << camera_frame << ", at time " << timestamp.toSec()
                    << ", in the tf tree.");
  }

  if (publish_feature_blocks_marker_) {
    CHECK_NOTNULL(feature_block_pub_);
    visualization_msgs::MarkerArray feature_blocks;
    // TODO(ntonci): Make this a param.
    constexpr size_t kMaxNumberOfFeatures = 500u;
    createOccupancyBlocksFromFeatureLayer(
        *feature_layer_, world_frame_, kMaxNumberOfFeatures, &feature_blocks);
    feature_block_pub_->publish(feature_blocks);
  }

  ros::WallTime end = ros::WallTime::now();
  ROS_INFO(
      "Integrated %lu features in %f seconds. Total number of features: %lu.",
      features_C.size(), (end - start).toSec(),
      feature_layer_->getNumberOfFeatures());
  ROS_INFO_STREAM("Timings: " << std::endl << timing::Timing::Print());
}

bool IodbController::publishSceneCallback(
    std_srvs::SetBool::Request& request,
    std_srvs::SetBool::Response& response) {
  bool save_scene_mesh = request.data;
  if (save_scene_mesh) {
    constexpr bool kClearMesh = true;
    generateMesh(kClearMesh);
  }
  publishScene();
  constexpr bool kPublishAllSegments = true;
  publishObjects(kPublishAllSegments);
  return true;
}

bool IodbController::validateMergedObjectCallback(
    modelify_msgs::ValidateMergedObject::Request& request,
    modelify_msgs::ValidateMergedObject::Response& response) {
  typedef TsdfVoxel TsdfVoxelType;
  typedef Layer<TsdfVoxelType> TsdfLayer;
  // TODO(ff): Do the following afterwards in modelify.
  // - Check if merged object agrees with whole map (at all poses).
  // - If it doesn't agree at all poses try the merging again with the
  // reduced set of objects.
  // - Optionally put the one that doesn't agree with
  // the others in a list not to merge with the other merged ones

  // Extract TSDF layer of merged object.
  std::shared_ptr<TsdfLayer> merged_object_layer_O;
  CHECK(deserializeMsgToLayer(request.gsm_update.object.tsdf_layer,
                              merged_object_layer_O.get()))
      << "Deserializing of TSDF layer from merged object message failed.";

  // Extract transformations.
  std::vector<Transformation> transforms_W_O;
  transformMsgs2Transformations(request.gsm_update.object.transforms,
                                &transforms_W_O);

  const utils::VoxelEvaluationMode voxel_evaluation_mode =
      utils::VoxelEvaluationMode::kEvaluateAllVoxels;

  std::vector<utils::VoxelEvaluationDetails> voxel_evaluation_details_vector;

  evaluateLayerRmseAtPoses<TsdfVoxelType>(
      voxel_evaluation_mode, map_->getTsdfLayer(),
      *(merged_object_layer_O.get()), transforms_W_O,
      &voxel_evaluation_details_vector);

  voxelEvaluationDetails2VoxelEvaluationDetailsMsg(
      voxel_evaluation_details_vector, &response.voxel_evaluation_details);
  return true;
}

void IodbController::publishGsmUpdate(const ros::Publisher& publisher,
                                      modelify_msgs::GsmUpdate* gsm_update) {
  publisher.publish(*gsm_update);
}

}  // namespace voxblox_gsm
}  // namespace voxblox
