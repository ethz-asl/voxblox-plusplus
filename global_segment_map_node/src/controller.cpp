// Copyright 2017 Margarita Grinvald, ASL, ETH Zurich, Switzerland

#include "voxblox_gsm/controller.h"

#include <cmath>
#include <string>
#include <unordered_set>

#include <global_segment_map/layer_evaluation.h>
#include <glog/logging.h>
#include <minkindr_conversions/kindr_tf.h>
#include <modelify/object_toolbox/object_toolbox.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/MarkerArray.h>
#include <voxblox/core/common.h>
#include <voxblox/integrator/merge_integration.h>
#include <voxblox/utils/layer_utils.h>
#include <voxblox_ros/conversions.h>
#include <voxblox_ros/mesh_vis.h>

#include <boost/filesystem.hpp>
#include <boost/thread.hpp>

#include "voxblox_gsm/conversions.h"

namespace voxblox {
namespace voxblox_gsm {

enum Method { kSimple = 0, kMerged, kMergedDiscard };
bool updatedMesh;
boost::mutex updateMeshMutex;

bool Controller::noNewUpdatesReceived(const double no_update_timeout) const {
  if (received_first_message_) {
    return (ros::Time::now() - last_update_received_).toSec() >
           no_update_timeout;
  }
  return false;
}

// TODO(grinvalm): make it more efficient by only updating the
// necessary polygons and not all of them each time.
void visualizeMesh(const MeshLayer& mesh_layer) {
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("GSM viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addCoordinateSystem(0.2);
  viewer->initCameraParameters();
  // TODO(grinvalm): find some general default parameters.
  viewer->setCameraPosition(1.4812, 0.338258, 0.96422, -0.238669, -0.785343,
                            -0.571204);
  viewer->setCameraClipDistances(0.0106586, 10.6586);

  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_ptr(&cloud);
  pcl::PointCloud<pcl::Normal> normals;
  pcl::PointCloud<pcl::Normal>::ConstPtr normals_ptr(&normals);

  while (!viewer->wasStopped()) {
    constexpr int updateIntervalms = 1000;
    viewer->spinOnce(updateIntervalms);

    boost::mutex::scoped_lock updatedMeshLock(updateMeshMutex);

    if (updatedMesh) {
      normals.points.clear();
      cloud.points.clear();

      Mesh combined_mesh(mesh_layer.block_size(), Point::Zero());

      // Combine everything in the layer into one giant combined mesh.
      size_t v = 0u;
      BlockIndexList mesh_indices;
      mesh_layer.getAllAllocatedMeshes(&mesh_indices);
      for (const BlockIndex& block_index : mesh_indices) {
        Mesh::ConstPtr mesh = mesh_layer.getMeshPtrByIndex(block_index);

        for (const Point& vert : mesh->vertices) {
          combined_mesh.vertices.push_back(vert);
          combined_mesh.indices.push_back(v++);
        }

        for (const Color& color : mesh->colors) {
          combined_mesh.colors.push_back(color);
        }

        for (const Point& normal : mesh->normals) {
          normals.points.push_back(
              pcl::Normal(normal(0), normal(1), normal(2)));
        }
      }

      size_t vert_idx = 0;
      for (const Point& vert : combined_mesh.vertices) {
        const Color& color = combined_mesh.colors[vert_idx];
        pcl::PointXYZRGB point = pcl::PointXYZRGB(color.r, color.g, color.b);
        point.x = vert(0);
        point.y = vert(1);
        point.z = vert(2);
        cloud.points.push_back(point);

        vert_idx++;
      }

      pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_with_normals;
      pcl::concatenateFields(cloud, normals, cloud_with_normals);

      pcl::PCLPointCloud2 pcl_pc;
      pcl::toPCLPointCloud2(cloud_with_normals, pcl_pc);

      std::vector<pcl::Vertices> polygons;

      for (size_t i = 0u; i < combined_mesh.indices.size(); i += 3u) {
        pcl::Vertices face;
        for (int j = 0; j < 3; j++) {
          face.vertices.push_back(combined_mesh.indices.at(i + j));
        }
        polygons.push_back(face);
      }

      pcl::PolygonMesh mesh;
      mesh.cloud = pcl_pc;
      mesh.polygons = polygons;

      viewer->removePolygonMesh("meshes");
      if (!viewer->updatePolygonMesh(mesh, "meshes")) {
        viewer->addPolygonMesh(mesh, "meshes", 0);
      }

      updatedMesh = false;
    }
    updatedMeshLock.unlock();
  }
}

Controller::Controller(ros::NodeHandle* node_handle_private)
    : node_handle_private_(node_handle_private),
      // Increased time limit for lookup in the past of tf messages
      // to give some slack to the pipeline and not lose any messages.
      tf_listener_(ros::Duration(100)),
      received_first_message_(false) {
  CHECK_NOTNULL(node_handle_private_);

  // map_config_.voxel_size = 0.005f;
  map_config_.voxel_size = 0.01f;
  map_config_.voxels_per_side = 8u;
  map_.reset(new LabelTsdfMap(map_config_));

  LabelTsdfIntegrator::Config integrator_config;
  integrator_config.voxel_carving_enabled = false;
  // integrator_config.voxel_carving_enabled = true;
  integrator_config.allow_clear = true;
  integrator_config.default_truncation_distance = map_config_.voxel_size * 5.0;
  integrator_config.max_ray_length_m = 2.5;

  std::string method("merged");
  node_handle_private_->param("method", method, method);
  if (method.compare("merged") == 0) {
    integrator_config.enable_anti_grazing = false;
  } else if (method.compare("merged_discard") == 0) {
    integrator_config.enable_anti_grazing = true;
  } else {
    integrator_config.enable_anti_grazing = false;
  }

  LabelTsdfIntegrator::LabelTsdfConfig label_tsdf_integrator_config;
  label_tsdf_integrator_config.enable_pairwise_confidence_merging = true;
  label_tsdf_integrator_config.pairwise_confidence_ratio_threshold =
      0.2f;  // 0.08 and 20 was ok until 1200
  label_tsdf_integrator_config.pairwise_confidence_threshold = 30;
  label_tsdf_integrator_config.object_flushing_age_threshold =
      30000;  // TODO(ff): For the real tests probably set to 30 or something.
  label_tsdf_integrator_config.cap_confidence = false;
  label_tsdf_integrator_config.confidence_cap_value = 10;

  integrator_.reset(new LabelTsdfIntegrator(
      integrator_config, label_tsdf_integrator_config, map_->getTsdfLayerPtr(),
      map_->getLabelLayerPtr(), map_->getHighestLabelPtr()));

  mesh_layer_.reset(new MeshLayer(map_->block_size()));
  mesh_integrator_.reset(
      new MeshLabelIntegrator(mesh_config_, map_->getTsdfLayerPtr(),
                              map_->getLabelLayerPtr(), mesh_layer_.get()));

  // If set, use a timer to progressively integrate the mesh.
  double update_mesh_every_n_sec = 1.0;

  if (update_mesh_every_n_sec > 0.0) {
    update_mesh_timer_ = node_handle_private_->createTimer(
        ros::Duration(update_mesh_every_n_sec), &Controller::updateMeshEvent,
        this);
  }

  integrated_frames_count_ = 0u;

  bool interactiveViewer = true;
  if (interactiveViewer) {
    boost::thread visualizerThread(visualizeMesh, boost::ref(*mesh_layer_));
  }
  ros::spinOnce();
}

Controller::~Controller() {}

void Controller::subscribeSegmentPointCloudTopic(
    ros::Subscriber* segment_point_cloud_sub) {
  CHECK_NOTNULL(segment_point_cloud_sub);
  // TODO(grinvalm): parametrize this.
  std::string segment_point_cloud_topic = "/scenenn_node/object_segment";
  node_handle_private_->param<std::string>("segment_point_cloud_topic",
                                           segment_point_cloud_topic,
                                           segment_point_cloud_topic);

  // Large queue size to give slack to the algorithm
  // pipeline and not lose any messages.
  *segment_point_cloud_sub = node_handle_private_->subscribe(
      segment_point_cloud_topic, 2000, &Controller::segmentPointCloudCallback,
      this);
}

void Controller::advertiseMeshTopic(ros::Publisher* mesh_pub) {
  CHECK_NOTNULL(mesh_pub);
  *mesh_pub = node_handle_private_->advertise<visualization_msgs::MarkerArray>(
      "mesh", 1, true);

  mesh_pub_ = mesh_pub;
}

void Controller::advertiseSceneTopic(ros::Publisher* scene_pub) {
  CHECK_NOTNULL(scene_pub);
  static const std::string kGsmSceneTopic = "scene";
  constexpr int kGsmSceneQueueSize = 1;

  *scene_pub = node_handle_private_->advertise<modelify_msgs::GsmUpdate>(
      kGsmSceneTopic, kGsmSceneQueueSize, true);
  scene_pub_ = scene_pub;
}

void Controller::advertiseObjectTopic(ros::Publisher* object_pub) {
  CHECK_NOTNULL(object_pub);
  *object_pub = node_handle_private_->advertise<visualization_msgs::Marker>(
      "segment_meshes", 1, true);

  object_pub_ = object_pub;
}

void Controller::advertiseGsmUpdateTopic(ros::Publisher* gsm_update_pub) {
  CHECK_NOTNULL(gsm_update_pub);
  static const std::string kGsmUpdateTopic = "gsm_update";
  // TODO(ff): Reduce this value, once we know some reasonable limit.
  constexpr int kGsmUpdateQueueSize = 2000;

  *gsm_update_pub = node_handle_private_->advertise<modelify_msgs::GsmUpdate>(
      kGsmUpdateTopic, kGsmUpdateQueueSize, true);
  gsm_update_pub_ = gsm_update_pub;
}

void Controller::validateMergedObjectService(
    ros::ServiceServer* validate_merged_object_srv) {
  CHECK_NOTNULL(validate_merged_object_srv);
  static const std::string kValidateMergedObjectTopicRosParam =
      "validate_merged_object";
  std::string validate_merged_object_topic = "validate_merged_object";
  node_handle_private_->param<std::string>(kValidateMergedObjectTopicRosParam,
                                           validate_merged_object_topic,
                                           validate_merged_object_topic);

  *validate_merged_object_srv = node_handle_private_->advertiseService(
      validate_merged_object_topic, &Controller::validateMergedObjectCallback,
      this);
}

void Controller::advertiseGenerateMeshService(
    ros::ServiceServer* generate_mesh_srv) {
  CHECK_NOTNULL(generate_mesh_srv);
  *generate_mesh_srv = node_handle_private_->advertiseService(
      "generate_mesh", &Controller::generateMeshCallback, this);
}

void Controller::advertisePublishSceneService(
    ros::ServiceServer* publish_scene_srv) {
  CHECK_NOTNULL(publish_scene_srv);
  static const std::string kAdvertisePublishSceneServiceName = "publish_scene";
  *publish_scene_srv = node_handle_private_->advertiseService(
      kAdvertisePublishSceneServiceName, &Controller::publishSceneCallback,
      this);
}

void Controller::advertiseExtractSegmentsService(
    ros::ServiceServer* extract_segments_srv) {
  CHECK_NOTNULL(extract_segments_srv);
  *extract_segments_srv = node_handle_private_->advertiseService(
      "extract_segments", &Controller::extractSegmentsCallback, this);
}

void Controller::publishScene() {
  CHECK_NOTNULL(scene_pub_);
  modelify_msgs::GsmUpdate gsm_update_msg;

  gsm_update_msg.header.stamp = last_segment_msg_timestamp_;
  static const std::string kGsmUpdateFrameId = "world";
  gsm_update_msg.header.frame_id = kGsmUpdateFrameId;

  constexpr bool kSerializeOnlyUpdated = false;
  serializeLayerAsMsg<TsdfVoxel>(map_->getTsdfLayer(), kSerializeOnlyUpdated,
                                 &gsm_update_msg.object.tsdf_layer);
  // TODO(ff): Make sure this works also, there is no LabelVoxel in voxblox
  // yet, hence it doesn't work.
  // serializeLayerAsMsg<LabelVoxel>(
  //     map_->label_layer_, kSerializeOnlyUpdated,
  //     &gsm_update_msg.object.label_layer);

  gsm_update_msg.object.label = 0u;
  gsm_update_msg.old_labels.clear();
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
  scene_pub_->publish(gsm_update_msg);
}

bool Controller::publishSceneCallback(std_srvs::Empty::Request& request,
                                      std_srvs::Empty::Response& response) {
  constexpr bool kClearMesh = true;
  generateMesh(kClearMesh);
  publishScene();
  constexpr bool kPublishAllSegments = true;
  publishObjects(kPublishAllSegments);
  return true;
}

void Controller::segmentPointCloudCallback(
    const sensor_msgs::PointCloud2::Ptr& segment_point_cloud_msg) {
  // Message timestamps are used to detect when all
  // segment messages from a certain frame have arrived.
  // Since segments from the same frame all have the same timestamp,
  // the start of a new frame is detected when the message timestamp changes.
  // TODO(grinvalm): need additional check for the last frame to be
  // integrated.
  if (received_first_message_ &&
      last_segment_msg_timestamp_ != segment_point_cloud_msg->header.stamp) {
    ROS_INFO("Integrating frame n.%zu, timestamp of frame: %f",
             ++integrated_frames_count_,
             segment_point_cloud_msg->header.stamp.toSec());
    ros::WallTime start = ros::WallTime::now();

    integrator_->decideLabelPointClouds(&segments_to_integrate_,
                                        &segment_label_candidates,
                                        &segment_merge_candidates_);

    ros::WallTime end = ros::WallTime::now();
    ROS_INFO("Decided labels for %lu pointclouds in %f seconds.",
             segments_to_integrate_.size(), (end - start).toSec());

    constexpr bool kIsFreespacePointcloud = false;

    start = ros::WallTime::now();

    for (const auto& segment : segments_to_integrate_) {
      integrator_->integratePointCloud(segment->T_G_C_, segment->points_C_,
                                       segment->colors_, segment->labels_,
                                       kIsFreespacePointcloud);
    }

    end = ros::WallTime::now();
    ROS_INFO("Finished integrating %lu pointclouds in %f seconds.",
             segments_to_integrate_.size(), (end - start).toSec());

    start = ros::WallTime::now();

    integrator_->mergeLabels(&merges_to_publish_);
    integrator_->getLabelsToPublish(&segment_labels_to_publish_);

    end = ros::WallTime::now();
    ROS_INFO(
        "Finished merging segments and fetching the ones to publish in %f "
        "seconds.",
        (end - start).toSec());
    start = ros::WallTime::now();

    segment_merge_candidates_.clear();
    segment_label_candidates.clear();
    for (Segment* segment : segments_to_integrate_) {
      delete segment;
    }
    segments_to_integrate_.clear();

    end = ros::WallTime::now();
    ROS_INFO("Cleared candidates and memory in %f seconds.",
             (end - start).toSec());

    // publishObjects();
  }
  received_first_message_ = true;
  last_update_received_ = ros::Time::now();
  last_segment_msg_timestamp_ = segment_point_cloud_msg->header.stamp;

  // Look up transform from camera frame to world frame.
  Transformation T_G_C;
  std::string world_frame_id = "world";
  // std::string world_frame_id = "/vicon";
  // std::string camera_frame_id = "/scenenet_camera_frame";
  std::string camera_frame_id = "/depth";
  // std::string camera_frame_id = "/camera_rgb_optical_frame";
  // TODO(grinvalm): nicely parametrize the frames.
  //  std::string camera_frame_id = segment_point_cloud_msg->header.frame_id;
  //  std::string camera_frame_id = "/openni_depth_optical_frame";
  //  std::string camera_frame_id = "/camera_depth_optical_frame";

  if (lookupTransform(camera_frame_id, world_frame_id,
                      segment_point_cloud_msg->header.stamp, &T_G_C)) {
    Segment* segment = new Segment();
    segments_to_integrate_.push_back(segment);

    // Convert the PCL pointcloud into voxblox format.
    // TODO(helenol): improve...
    // Horrible hack fix to fix color parsing colors in PCL.
    for (size_t d = 0; d < segment_point_cloud_msg->fields.size(); ++d) {
      if (segment_point_cloud_msg->fields[d].name == std::string("rgb")) {
        segment_point_cloud_msg->fields[d].datatype =
            sensor_msgs::PointField::FLOAT32;
      }
    }

    modelify::PointCloudType point_cloud;
    pcl::fromROSMsg(*segment_point_cloud_msg, point_cloud);

    // Filter out NaNs. :|
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(point_cloud, point_cloud, indices);

    for (size_t i = 0; i < point_cloud.points.size(); ++i) {
      if (std::isnan(point_cloud.points[i].x) ||
          std::isnan(point_cloud.points[i].y) ||
          std::isnan(point_cloud.points[i].z)) {
        continue;
      }

      segment->points_C_.push_back(Point(point_cloud.points[i].x,
                                         point_cloud.points[i].y,
                                         point_cloud.points[i].z));

      segment->colors_.push_back(
          Color(point_cloud.points[i].r, point_cloud.points[i].g,
                point_cloud.points[i].b, point_cloud.points[i].a));
    }

    segment->T_G_C_ = T_G_C;

    ros::WallTime start = ros::WallTime::now();
    integrator_->computeSegmentLabelCandidates(
        segment, &segment_label_candidates, &segment_merge_candidates_);

    ros::WallTime end = ros::WallTime::now();
    ROS_INFO(
        "Computed label candidates for a pointcloud of size %lu in %f seconds.",
        segment->points_C_.size(), (end - start).toSec());

    ROS_INFO_STREAM("Timings: " << std::endl << timing::Timing::Print());
  }
}

bool Controller::validateMergedObjectCallback(
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
  voxblox_gsm::transformMsgs2Transformations(
      request.gsm_update.object.transforms, &transforms_W_O);

  const utils::VoxelEvaluationMode voxel_evaluation_mode =
      utils::VoxelEvaluationMode::kEvaluateAllVoxels;

  std::vector<utils::VoxelEvaluationDetails> voxel_evaluation_details_vector;

  evaluateLayerAtPoses<TsdfVoxelType>(
      voxel_evaluation_mode, map_->getTsdfLayer(),
      *(merged_object_layer_O.get()), transforms_W_O,
      &voxel_evaluation_details_vector);

  voxblox_gsm::voxelEvaluationDetails2VoxelEvaluationDetailsMsg(
      voxel_evaluation_details_vector, &response.voxel_evaluation_details);
  return true;
}

void Controller::generateMesh(bool clear_mesh) {  // NOLINT
  voxblox::timing::Timer generate_mesh_timer("mesh/generate");
  boost::mutex::scoped_lock updateMeshLock(updateMeshMutex);
  if (clear_mesh) {
    constexpr bool only_mesh_updated_blocks = false;
    constexpr bool clear_updated_flag = true;
    mesh_integrator_->generateMesh(only_mesh_updated_blocks,
                                   clear_updated_flag);
  } else {
    constexpr bool only_mesh_updated_blocks = true;
    constexpr bool clear_updated_flag = true;
    mesh_integrator_->generateMesh(only_mesh_updated_blocks,
                                   clear_updated_flag);
  }
  generate_mesh_timer.Stop();

  // timing::Timer publish_mesh_timer("mesh/publish");
  // visualization_msgs::MarkerArray marker_array;
  // marker_array.markers.resize(1);
  // fillMarkerWithMesh(mesh_layer_, ColorMode::kColor,
  // &marker_array.markers[0]);
  // // TODO(grinvalm): world frame param.
  // ColorMode color_mode_ = ColorMode::kColor;
  // std::string world_frame_ = "world";
  // marker_array.markers[0].header.frame_id = world_frame_;
  // mesh_pub_->publish(marker_array);
  //
  // publish_mesh_timer.Stop();

  // TODO(grinvalm): param.
  std::string mesh_filename_ = "voxblox_gsm_mesh.ply";

  if (!mesh_filename_.empty()) {
    timing::Timer output_mesh_timer("mesh/output");
    bool success = outputMeshLayerAsPly(mesh_filename_, false, *mesh_layer_);
    output_mesh_timer.Stop();
    if (success) {
      ROS_INFO("Output file as PLY: %s", mesh_filename_.c_str());
    } else {
      ROS_INFO("Failed to output mesh as PLY: %s", mesh_filename_.c_str());
    }
  }
  // TODO(grinvalm): flag rename or remove?

  updatedMesh = true;
  updateMeshLock.unlock();

  ROS_INFO_STREAM("Mesh Timings: " << std::endl
                                   << voxblox::timing::Timing::Print());
}

bool Controller::generateMeshCallback(
    std_srvs::Empty::Request& request,
    std_srvs::Empty::Response& response) {  // NOLINT
  constexpr bool kClearMesh = true;
  generateMesh(kClearMesh);
  return true;
}

void Controller::updateMeshEvent(const ros::TimerEvent& e) {
  boost::mutex::scoped_lock updateMeshLock(updateMeshMutex);
  bool updated = false;

  timing::Timer generate_mesh_timer("mesh/update");
  constexpr bool only_mesh_updated_blocks = true;
  constexpr bool clear_updated_flag = true;
  updatedMesh = mesh_integrator_->generateMesh(only_mesh_updated_blocks,
                                               clear_updated_flag);
  // updatedMesh = true;
  updateMeshLock.unlock();

  generate_mesh_timer.Stop();

  // // TODO(helenol): also think about how to update markers incrementally?
  // timing::Timer publish_mesh_timer("mesh/publish");
  // visualization_msgs::MarkerArray marker_array;
  // marker_array.markers.resize(1);
  // fillMarkerWithMesh(mesh_layer_, ColorMode::kColor,
  // &marker_array.markers[0]); marker_array.markers[0].header.frame_id =
  // "world"; mesh_pub_->publish(marker_array);

  // publish_mesh_timer.Stop();
}

bool Controller::extractSegmentsCallback(std_srvs::Empty::Request& request,
                                         std_srvs::Empty::Response& response) {
  // Get list of all labels in the map.
  std::vector<Label> labels = integrator_->getLabelsList();

  // TODO(grinvalm): there are more segments extracted than there really are.
  // Seems like their voxel count is greater than 0, but when meshed they are
  // too small to result in any polygon. Find a fix, maybe with a size
  // threshold.
  for (Label label : labels) {
    Layer<TsdfVoxel> tsdf_layer(map_config_.voxel_size,
                                map_config_.voxels_per_side);
    Layer<LabelVoxel> label_layer(map_config_.voxel_size,
                                  map_config_.voxels_per_side);
    // Extract the TSDF and label layers corresponding to a segment.
    extractSegmentLayers(label, &tsdf_layer, &label_layer);

    MeshLayer mesh_layer(map_->block_size());
    MeshLabelIntegrator mesh_integrator(mesh_config_, &tsdf_layer, &label_layer,
                                        &mesh_layer);

    constexpr bool only_mesh_updated_blocks = false;
    constexpr bool clear_updated_flag = true;
    mesh_integrator.generateMesh(only_mesh_updated_blocks, clear_updated_flag);

    Mesh::Ptr combined_mesh =
        aligned_shared<Mesh>(mesh_layer.block_size(), Point::Zero());
    mesh_layer.getMesh(combined_mesh);

    if (combined_mesh->vertices.size() > 0) {
      boost::filesystem::path segments_dir("segments");
      boost::filesystem::create_directory(segments_dir);

      std::string mesh_filename =
          "segments/voxblox_gsm_mesh_label_" + std::to_string(label) + ".ply";

      bool success = outputMeshAsPly(mesh_filename, *combined_mesh);
      if (success) {
        ROS_INFO("Output segment file as PLY: %s", mesh_filename.c_str());
      } else {
        ROS_INFO("Failed to output mesh as PLY: %s", mesh_filename.c_str());
      }
    }
  }
  return true;
}

void Controller::extractSegmentLayers(Label label, Layer<TsdfVoxel>* tsdf_layer,
                                      Layer<LabelVoxel>* label_layer) {
  CHECK_NOTNULL(tsdf_layer);
  CHECK_NOTNULL(label_layer);
  // TODO(grinvalm): find a less naive method to
  // extract all blocks and voxels for a label.
  BlockIndexList all_label_blocks;
  map_->getTsdfLayerPtr()->getAllAllocatedBlocks(&all_label_blocks);

  for (const BlockIndex& block_index : all_label_blocks) {
    Block<TsdfVoxel>::Ptr global_tsdf_block =
        map_->getTsdfLayerPtr()->getBlockPtrByIndex(block_index);
    Block<LabelVoxel>::Ptr global_label_block =
        map_->getLabelLayerPtr()->getBlockPtrByIndex(block_index);
    Block<TsdfVoxel>::Ptr tsdf_block;
    Block<LabelVoxel>::Ptr label_block;

    size_t vps = global_label_block->voxels_per_side();
    for (int i = 0; i < vps * vps * vps; i++) {
      const LabelVoxel& global_label_voxel =
          global_label_block->getVoxelByLinearIndex(i);

      if (global_label_voxel.label != label) {
        continue;
      }
      if (!tsdf_block) {
        tsdf_block = tsdf_layer->allocateBlockPtrByIndex(block_index);
        label_block = label_layer->allocateBlockPtrByIndex(block_index);
      }
      CHECK(tsdf_block);
      CHECK(label_block);

      TsdfVoxel& tsdf_voxel = tsdf_block->getVoxelByLinearIndex(i);
      LabelVoxel& label_voxel = label_block->getVoxelByLinearIndex(i);

      const TsdfVoxel& global_tsdf_voxel =
          global_tsdf_block->getVoxelByLinearIndex(i);

      tsdf_voxel = global_tsdf_voxel;
      label_voxel = global_label_voxel;
    }
  }
}

// TODO(ff): Create this somewhere:
// void serializeGsmAsMsg(const map&, const label&, const parent_labels&, msg*);

void Controller::publishObjects(const bool publish_all) {
  CHECK_NOTNULL(gsm_update_pub_);
  // TODO(ff): Not sure if we want to use this or ros::Time::now();
  const std::vector<Label>* labels_to_publish_ptr = &segment_labels_to_publish_;
  std::vector<Label> all_labels;
  if (publish_all) {
    all_labels = integrator_->getLabelsList();
    labels_to_publish_ptr = &all_labels;
    LOG(INFO) << "Publishing all segments";
  }

  for (const Label& label : *labels_to_publish_ptr) {
    // Extract the TSDF and label layers corresponding to this label.
    Layer<TsdfVoxel> tsdf_layer(map_config_.voxel_size,
                                map_config_.voxels_per_side);
    Layer<LabelVoxel> label_layer(map_config_.voxel_size,
                                  map_config_.voxels_per_side);
    extractSegmentLayers(label, &tsdf_layer, &label_layer);

    // Continue if tsdf_layer has little getNumberOfAllocatedBlocks.
    // TODO(ff): Check what a reasonable size is for this.
    constexpr size_t kMinNumberOfAllocatedBlocksToPublish = 10u;
    if (tsdf_layer.getNumberOfAllocatedBlocks() <
        kMinNumberOfAllocatedBlocksToPublish) {
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

    // Extract surfel cloud from layer.
    pcl::PointCloud<pcl::PointSurfel>::Ptr surfel_cloud(
        new pcl::PointCloud<pcl::PointSurfel>());
    convertVoxelGridToPointCloud(tsdf_layer, surfel_cloud.get());

    if (surfel_cloud->empty()) {
      LOG(WARNING) << "Labelled segment does not contain enough data to "
                      "extract a surface -> skipping!";
      continue;
    }

    modelify_msgs::GsmUpdate gsm_update_msg;
    constexpr bool kSerializeOnlyUpdated = false;
    gsm_update_msg.header.stamp = last_segment_msg_timestamp_;
    static const std::string kGsmUpdateFrameId = "world";
    gsm_update_msg.header.frame_id = kGsmUpdateFrameId;
    serializeLayerAsMsg<TsdfVoxel>(tsdf_layer, kSerializeOnlyUpdated,
                                   &gsm_update_msg.object.tsdf_layer);
    // TODO(ff): Make sure this works also, there is no LabelVoxel in voxblox
    // yet, hence it doesn't work.
    // serializeLayerAsMsg<LabelVoxel>(
    //     label_layer, kSerializeOnlyUpdated,
    //     &gsm_update_msg.object.label_layer);

    gsm_update_msg.object.label = label;
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
    gsm_update_pub_->publish(gsm_update_msg);
    // TODO(ff): Fill in gsm_update_msg.object.surfel_cloud if needed.

    // Generate mesh for visualization purposes.
    std::shared_ptr<MeshLayer> mesh_layer;
    mesh_layer.reset(new MeshLayer(tsdf_layer.block_size()));
    // mesh_layer.reset(new MeshLayer(map_->block_size()));
    MeshLabelIntegrator mesh_integrator(mesh_config_, &tsdf_layer, &label_layer,
                                        mesh_layer.get());
    constexpr bool only_mesh_updated_blocks = false;
    constexpr bool clear_updated_flag = true;
    mesh_integrator.generateMesh(only_mesh_updated_blocks, clear_updated_flag);

    visualization_msgs::Marker segment_marker;
    fillMarkerWithMesh(mesh_layer, ColorMode::kColor, &segment_marker);
    static const std::string kWorldFrameName = "world";
    segment_marker.header.frame_id = kWorldFrameName;
    if (segment_marker.points.size() > 0) {
      object_pub_->publish(segment_marker);
    }
    all_published_segments_.insert(label);
  }
  segment_labels_to_publish_.clear();
}

bool Controller::lookupTransform(const std::string& from_frame,
                                 const std::string& to_frame,
                                 const ros::Time& timestamp,
                                 Transformation* transform) {
  tf::StampedTransform tf_transform;
  ros::Time time_to_lookup = timestamp;

  // If this transform isn't possible at the time, then try to just look
  // up the latest (this is to work with bag files and static transform
  // publisher, etc).
  if (!tf_listener_.canTransform(to_frame, from_frame, time_to_lookup)) {
    return false;
    time_to_lookup = ros::Time(0);
    LOG(ERROR) << "Using latest TF transform instead of timestamp match.";
  }

  try {
    tf_listener_.lookupTransform(to_frame, from_frame, time_to_lookup,
                                 tf_transform);
  } catch (tf::TransformException& ex) {
    LOG(ERROR) << "Error getting TF transform from sensor data: " << ex.what();
    return false;
  }

  tf::transformTFToKindr(tf_transform, transform);
  return true;
}

}  // namespace voxblox_gsm
}  // namespace voxblox
