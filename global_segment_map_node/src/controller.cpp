// Copyright (c) 2019, ASL, ETH Zurich, Switzerland
// Licensed under the BSD 3-Clause License (see LICENSE for details)

// TODO(ntonci): Fix file extension. These files in global_segment_map_node have
// cpp extension and all others have cc.

#include "global_segment_map_node/controller.h"

#include <stdlib.h>

#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <geometry_msgs/TransformStamped.h>
#include <global_segment_map/label_voxel.h>
#include <global_segment_map/utils/file_utils.h>
#include <global_segment_map/utils/map_utils.h>
#include <glog/logging.h>
#include <minkindr_conversions/kindr_tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <voxblox/alignment/icp.h>
#include <voxblox/core/common.h>
#include <voxblox/io/sdf_ply.h>
#include <voxblox_ros/mesh_vis.h>
#include "global_segment_map_node/conversions.h"

#ifdef APPROXMVBB_AVAILABLE
#include <ApproxMVBB/ComputeApproxMVBB.hpp>
#endif

namespace voxblox {
namespace voxblox_gsm {

// TODO(ntonci): Move this to a separate header.
std::string classes[81] = {"BG",
                           "person",
                           "bicycle",
                           "car",
                           "motorcycle",
                           "airplane",
                           "bus",
                           "train",
                           "truck",
                           "boat",
                           "traffic light",
                           "fire hydrant",
                           "stop sign",
                           "parking meter",
                           "bench",
                           "bird",
                           "cat",
                           "dog",
                           "horse",
                           "sheep",
                           "cow",
                           "elephant",
                           "bear",
                           "zebra",
                           "giraffe",
                           "backpack",
                           "umbrella",
                           "handbag",
                           "tie",
                           "suitcase",
                           "frisbee",
                           "skis",
                           "snowboard",
                           "sports ball",
                           "kite",
                           "baseball bat",
                           "baseball glove",
                           "skateboard",
                           "surfboard",
                           "tennis racket",
                           "bottle",
                           "wine glass",
                           "cup",
                           "fork",
                           "knife",
                           "spoon",
                           "bowl",
                           "banana",
                           "apple",
                           "sandwich",
                           "orange",
                           "broccoli",
                           "carrot",
                           "hot dog",
                           "pizza",
                           "donut",
                           "cake",
                           "chair",
                           "couch",
                           "potted plant",
                           "bed",
                           "dining table",
                           "toilet",
                           "tv",
                           "laptop",
                           "mouse",
                           "remote",
                           "keyboard",
                           "cell phone",
                           "microwave",
                           "oven",
                           "toaster",
                           "sink",
                           "refrigerator",
                           "book",
                           "clock",
                           "vase",
                           "scissors",
                           "teddy bear",
                           "hair drier",
                           "toothbrush"};

Controller::Controller(ros::NodeHandle* node_handle_private)
    : node_handle_private_(node_handle_private),
      // Increased time limit for lookup in the past of tf messages
      // to give some slack to the pipeline and not lose any messages.
      integrated_frames_count_(0u),
      tf_listener_(ros::Duration(500)),
      world_frame_("world"),
      integration_on_(true),
      publish_scene_map_(false),
      publish_scene_mesh_(false),
      received_first_message_(false),
      mesh_layer_updated_(false),
      need_full_remesh_(false),
      enable_semantic_instance_segmentation_(true),
      publish_object_bbox_(false),
      use_label_propagation_(true) {
  CHECK_NOTNULL(node_handle_private_);

  bool verbose_log = false;
  node_handle_private_->param<bool>("debug/verbose_log", verbose_log,
                                    verbose_log);

  if (verbose_log) {
    FLAGS_stderrthreshold = 0;
  }

  node_handle_private_->param<std::string>("world_frame_id", world_frame_,
                                           world_frame_);

  // Workaround for OS X on mac mini not having specializations for float
  // for some reason.
  int voxels_per_side = map_config_.voxels_per_side;
  node_handle_private_->param<FloatingPoint>(
      "voxblox/voxel_size", map_config_.voxel_size, map_config_.voxel_size);
  node_handle_private_->param<int>("voxblox/voxels_per_side", voxels_per_side,
                                   voxels_per_side);
  if (!isPowerOfTwo(voxels_per_side)) {
    LOG(ERROR)
        << "voxels_per_side must be a power of 2, setting to default value.";
    voxels_per_side = map_config_.voxels_per_side;
  }
  map_config_.voxels_per_side = voxels_per_side;

  map_.reset(new LabelTsdfMap(map_config_));

  // Determine TSDF integrator parameters.
  tsdf_integrator_config_.voxel_carving_enabled = false;
  tsdf_integrator_config_.allow_clear = true;
  FloatingPoint truncation_distance_factor = 5.0f;
  tsdf_integrator_config_.max_ray_length_m = 4.0f;

  node_handle_private_->param<bool>(
      "voxblox/voxel_carving_enabled",
      tsdf_integrator_config_.voxel_carving_enabled,
      tsdf_integrator_config_.voxel_carving_enabled);
  node_handle_private_->param<bool>("voxblox/allow_clear",
                                    tsdf_integrator_config_.allow_clear,
                                    tsdf_integrator_config_.allow_clear);
  node_handle_private_->param<FloatingPoint>(
      "voxblox/truncation_distance_factor", truncation_distance_factor,
      truncation_distance_factor);
  node_handle_private_->param<FloatingPoint>(
      "voxblox/min_ray_length_m", tsdf_integrator_config_.min_ray_length_m,
      tsdf_integrator_config_.min_ray_length_m);
  node_handle_private_->param<FloatingPoint>(
      "voxblox/max_ray_length_m", tsdf_integrator_config_.max_ray_length_m,
      tsdf_integrator_config_.max_ray_length_m);

  tsdf_integrator_config_.default_truncation_distance =
      map_config_.voxel_size * truncation_distance_factor;

  std::string method("merged");
  node_handle_private_->param<std::string>("method", method, method);
  if (method.compare("merged") == 0) {
    tsdf_integrator_config_.enable_anti_grazing = false;
  } else if (method.compare("merged_discard") == 0) {
    tsdf_integrator_config_.enable_anti_grazing = true;
  } else {
    tsdf_integrator_config_.enable_anti_grazing = false;
  }

  // Determine label integrator parameters.
  node_handle_private_->param<bool>(
      "pairwise_confidence_merging/enable_pairwise_confidence_merging",
      label_tsdf_integrator_config_.enable_pairwise_confidence_merging,
      label_tsdf_integrator_config_.enable_pairwise_confidence_merging);
  node_handle_private_->param<FloatingPoint>(
      "pairwise_confidence_merging/merging_min_overlap_ratio",
      label_tsdf_integrator_config_.merging_min_overlap_ratio,
      label_tsdf_integrator_config_.merging_min_overlap_ratio);
  node_handle_private_->param<int>(
      "pairwise_confidence_merging/merging_min_frame_count",
      label_tsdf_integrator_config_.merging_min_frame_count,
      label_tsdf_integrator_config_.merging_min_frame_count);

  node_handle_private_->param<bool>(
      "semantic_instance_segmentation/enable_semantic_instance_segmentation",
      label_tsdf_integrator_config_.enable_semantic_instance_segmentation,
      label_tsdf_integrator_config_.enable_semantic_instance_segmentation);

  enable_semantic_instance_segmentation_ =
      label_tsdf_integrator_config_.enable_semantic_instance_segmentation;

  std::string class_task("coco80");
  node_handle_private_->param<std::string>(
      "semantic_instance_segmentation/class_task", class_task, class_task);
  if (class_task.compare("coco80") == 0) {
    label_tsdf_mesh_config_.class_task = SemanticColorMap::ClassTask ::kCoco80;
  } else if (class_task.compare("nyu13") == 0) {
    label_tsdf_mesh_config_.class_task = SemanticColorMap::ClassTask ::kNyu13;
  } else {
    label_tsdf_mesh_config_.class_task = SemanticColorMap::ClassTask::kCoco80;
  }

  node_handle_private_->param<bool>("icp/enable_icp",
                                    label_tsdf_integrator_config_.enable_icp,
                                    label_tsdf_integrator_config_.enable_icp);
  node_handle_private_->param<bool>(
      "icp/keep_track_of_icp_correction",
      label_tsdf_integrator_config_.keep_track_of_icp_correction,
      label_tsdf_integrator_config_.keep_track_of_icp_correction);

  integrator_.reset(new LabelTsdfIntegrator(
      tsdf_integrator_config_, label_tsdf_integrator_config_, map_.get()));

  // Visualization settings.
  bool visualize = false;
  node_handle_private_->param<bool>("meshing/visualize", visualize, visualize);

  bool save_visualizer_frames = false;
  node_handle_private_->param<bool>("debug/save_visualizer_frames",
                                    save_visualizer_frames,
                                    save_visualizer_frames);

  bool multiple_visualizers = false;
  node_handle_private_->param<bool>("debug/multiple_visualizers",
                                    multiple_visualizers_,
                                    multiple_visualizers_);

  mesh_merged_layer_.reset(new MeshLayer(map_->block_size()));

  if (multiple_visualizers_) {
    mesh_label_layer_.reset(new MeshLayer(map_->block_size()));
    mesh_semantic_layer_.reset(new MeshLayer(map_->block_size()));
    mesh_instance_layer_.reset(new MeshLayer(map_->block_size()));
  }

  resetMeshIntegrators();

  std::vector<double> camera_position;
  std::vector<double> clip_distances;
  node_handle_private_->param<std::vector<double>>(
      "meshing/visualizer_parameters/camera_position", camera_position,
      camera_position);
  node_handle_private_->param<std::vector<double>>(
      "meshing/visualizer_parameters/clip_distances", clip_distances,
      clip_distances);
  if (visualize) {
    std::vector<std::shared_ptr<MeshLayer>> mesh_layers;

    mesh_layers.push_back(mesh_merged_layer_);

    if (multiple_visualizers_) {
      mesh_layers.push_back(mesh_label_layer_);
      mesh_layers.push_back(mesh_instance_layer_);
      mesh_layers.push_back(mesh_semantic_layer_);
    }

    visualizer_ =
        new Visualizer(mesh_layers, &mesh_layer_updated_, &mesh_layer_mutex_,
                       camera_position, clip_distances, save_visualizer_frames);
    viz_thread_ = std::thread(&Visualizer::visualizeMesh, visualizer_);
  }

  node_handle_private_->param<bool>("publishers/publish_scene_map",
                                    publish_scene_map_, publish_scene_map_);
  node_handle_private_->param<bool>("publishers/publish_scene_mesh",
                                    publish_scene_mesh_, publish_scene_mesh_);
  node_handle_private_->param<bool>("publishers/publish_object_bbox",
                                    publish_object_bbox_, publish_object_bbox_);

  node_handle_private_->param<bool>(
      "use_label_propagation", use_label_propagation_, use_label_propagation_);

  // If set, use a timer to progressively update the mesh.
  double update_mesh_every_n_sec = 0.0;
  node_handle_private_->param<double>("meshing/update_mesh_every_n_sec",
                                      update_mesh_every_n_sec,
                                      update_mesh_every_n_sec);

  if (update_mesh_every_n_sec > 0.0) {
    update_mesh_timer_ = node_handle_private_->createTimer(
        ros::Duration(update_mesh_every_n_sec), &Controller::updateMeshEvent,
        this);
  }

  node_handle_private_->param<std::string>("meshing/mesh_filename",
                                           mesh_filename_, mesh_filename_);
}

Controller::~Controller() { viz_thread_.join(); }

void Controller::subscribeSegmentPointCloudTopic(
    ros::Subscriber* segment_point_cloud_sub) {
  CHECK_NOTNULL(segment_point_cloud_sub);
  std::string segment_point_cloud_topic =
      "/depth_segmentation_node/object_segment";
  node_handle_private_->param<std::string>("segment_point_cloud_topic",
                                           segment_point_cloud_topic,
                                           segment_point_cloud_topic);
  // TODO (margaritaG): make this a param once segments of a frame are
  // refactored to be received as one single message.
  // Large queue size to give slack to the
  // pipeline and not lose any messages.
  constexpr int kSegmentPointCloudQueueSize = 6000;
  *segment_point_cloud_sub = node_handle_private_->subscribe(
      segment_point_cloud_topic, kSegmentPointCloudQueueSize,
      &Controller::segmentPointCloudCallback, this);
}

void Controller::advertiseMapTopic() {
  map_cloud_pub_ = new ros::Publisher(
      node_handle_private_->advertise<pcl::PointCloud<PointMapType>>("map", 1,
                                                                     true));
}

void Controller::advertiseSceneMeshTopic() {
  scene_mesh_pub_ = new ros::Publisher(
      node_handle_private_->advertise<voxblox_msgs::Mesh>("mesh", 1, true));
}

void Controller::advertiseSceneCloudTopic() {
  scene_cloud_pub_ = new ros::Publisher(
      node_handle_private_->advertise<pcl::PointCloud<pcl::PointXYZRGB>>(
          "cloud", 1, true));
}

void Controller::advertiseBboxTopic() {
  bbox_pub_ = new ros::Publisher(
      node_handle_private_->advertise<visualization_msgs::Marker>("bbox", 1,
                                                                  true));
}

void Controller::advertiseResetMapService(ros::ServiceServer* reset_map_srv) {
  CHECK_NOTNULL(reset_map_srv);
  *reset_map_srv = node_handle_private_->advertiseService(
      "reset_map", &Controller::resetMapCallback, this);
}

void Controller::advertiseToggleIntegrationService(
    ros::ServiceServer* toggle_integration_srv) {
  CHECK_NOTNULL(toggle_integration_srv);
  *toggle_integration_srv = node_handle_private_->advertiseService(
      "toggle_integration", &Controller::toggleIntegrationCallback, this);
}

void Controller::advertiseGetMapService(ros::ServiceServer* get_map_srv) {
  CHECK_NOTNULL(get_map_srv);
  *get_map_srv = node_handle_private_->advertiseService(
      "get_map", &Controller::getMapCallback, this);
}

void Controller::advertiseGenerateMeshService(
    ros::ServiceServer* generate_mesh_srv) {
  CHECK_NOTNULL(generate_mesh_srv);
  *generate_mesh_srv = node_handle_private_->advertiseService(
      "generate_mesh", &Controller::generateMeshCallback, this);
}

void Controller::advertiseGetScenePointcloudService(
    ros::ServiceServer* get_scene_pointcloud) {
  CHECK_NOTNULL(get_scene_pointcloud);
  *get_scene_pointcloud = node_handle_private_->advertiseService(
      "get_scene_pointcloud", &Controller::getScenePointcloudCallback, this);
}

void Controller::advertiseSaveSegmentsAsMeshService(
    ros::ServiceServer* save_segments_as_mesh_srv) {
  CHECK_NOTNULL(save_segments_as_mesh_srv);
  *save_segments_as_mesh_srv = node_handle_private_->advertiseService(
      "save_segments_as_mesh", &Controller::saveSegmentsAsMeshCallback, this);
}

void Controller::advertiseExtractInstancesService(
    ros::ServiceServer* extract_instances_srv) {
  CHECK_NOTNULL(extract_instances_srv);
  *extract_instances_srv = node_handle_private_->advertiseService(
      "extract_instances", &Controller::extractInstancesCallback, this);
}

void Controller::advertiseGetListSemanticInstancesService(
    ros::ServiceServer* get_list_semantic_instances_srv) {
  CHECK_NOTNULL(get_list_semantic_instances_srv);
  *get_list_semantic_instances_srv = node_handle_private_->advertiseService(
      "get_list_semantic_instances",
      &Controller::getListSemanticInstancesCallback, this);
}

void Controller::advertiseGetAlignedInstanceBoundingBoxService(
    ros::ServiceServer* get_instance_bounding_box_srv) {
  CHECK_NOTNULL(get_instance_bounding_box_srv);
  *get_instance_bounding_box_srv = node_handle_private_->advertiseService(
      "get_aligned_instance_bbox",
      &Controller::getAlignedInstanceBoundingBoxCallback, this);
}

void Controller::processSegment(
    const sensor_msgs::PointCloud2::Ptr& segment_point_cloud_msg) {
  // Look up transform from camera frame to world frame.
  Transformation T_G_C;
  std::string from_frame = segment_point_cloud_msg->header.frame_id;
  if (lookupTransform(from_frame, world_frame_,
                      segment_point_cloud_msg->header.stamp, &T_G_C)) {
    // Convert the PCL pointcloud into voxblox format.
    // Horrible hack fix to fix color parsing colors in PCL.
    for (size_t d = 0; d < segment_point_cloud_msg->fields.size(); ++d) {
      if (segment_point_cloud_msg->fields[d].name == std::string("rgb")) {
        segment_point_cloud_msg->fields[d].datatype =
            sensor_msgs::PointField::FLOAT32;
      }
    }
    timing::Timer ptcloud_timer("ptcloud_preprocess");

    Segment* segment = nullptr;
    if (enable_semantic_instance_segmentation_) {
      pcl::PointCloud<voxblox::PointSemanticInstanceType>
          point_cloud_semantic_instance;
      pcl::fromROSMsg(*segment_point_cloud_msg, point_cloud_semantic_instance);
      segment = new Segment(point_cloud_semantic_instance, T_G_C);
    } else if (use_label_propagation_) {
      // TODO(ntonci): maybe rename use_label_propagation_ to something like
      // use_voxblox_plus_plus_lables_ or change the order and call it
      // use_external_labels_
      pcl::PointCloud<voxblox::PointType> point_cloud;
      pcl::fromROSMsg(*segment_point_cloud_msg, point_cloud);
      segment = new Segment(point_cloud, T_G_C);
    } else {
      pcl::PointCloud<voxblox::PointLabelType> point_cloud_label;
      pcl::fromROSMsg(*segment_point_cloud_msg, point_cloud_label);
      segment = new Segment(point_cloud_label, T_G_C);
    }
    CHECK_NOTNULL(segment);
    segments_to_integrate_.push_back(segment);
    ptcloud_timer.Stop();

    timing::Timer label_candidates_timer("compute_label_candidates");

    if (use_label_propagation_) {
      integrator_->computeSegmentLabelCandidates(
          segment, &segment_label_candidates, &segment_merge_candidates_);
    }
  }
}

void Controller::integrateFrame(ros::Time msg_timestamp) {
  LOG(INFO) << "Integrating frame n." << ++integrated_frames_count_
            << ", timestamp of frame: " << msg_timestamp.toSec();
  ros::WallTime start;
  ros::WallTime end;

  if (use_label_propagation_) {
    start = ros::WallTime::now();
    timing::Timer propagation_timer("label_propagation");

    integrator_->decideLabelPointClouds(&segments_to_integrate_,
                                        &segment_label_candidates,
                                        &segment_merge_candidates_);
    propagation_timer.Stop();
    end = ros::WallTime::now();
    LOG(INFO) << "Decided labels for " << segments_to_integrate_.size()
              << " pointclouds in " << (end - start).toSec() << " seconds.";
  }

  constexpr bool kIsFreespacePointcloud = false;

  start = ros::WallTime::now();
  timing::Timer integrate_timer("integrate_frame_pointclouds");
  Transformation T_G_C = segments_to_integrate_.at(0)->T_G_C_;
  Pointcloud point_cloud_all_segments_t;
  for (Segment* segment : segments_to_integrate_) {
    // Concatenate point clouds. (NOTE(ff): We should probably just use
    // the original cloud here instead.)
    Pointcloud::iterator it = point_cloud_all_segments_t.end();
    point_cloud_all_segments_t.insert(it, segment->points_C_.begin(),
                                      segment->points_C_.end());
  }
  Transformation T_Gicp_C = T_G_C;
  if (label_tsdf_integrator_config_.enable_icp) {
    // TODO(ntonci): Make icp config members ros params.
    // integrator_->icp_.reset(new
    // ICP(getICPConfigFromRosParam(nh_private)));
    T_Gicp_C =
        integrator_->getIcpRefined_T_G_C(T_G_C, point_cloud_all_segments_t);
  }

  {
    std::lock_guard<std::mutex> label_tsdf_layers_lock(
        label_tsdf_layers_mutex_);
    for (Segment* segment : segments_to_integrate_) {
      CHECK_NOTNULL(segment);
      segment->T_G_C_ = T_Gicp_C;

      integrator_->integratePointCloud(segment->T_G_C_, segment->points_C_,
                                       segment->colors_, segment->label_,
                                       kIsFreespacePointcloud);
    }
  }

  integrate_timer.Stop();
  end = ros::WallTime::now();
  LOG(INFO) << "Integrated " << segments_to_integrate_.size()
            << " pointclouds in " << (end - start).toSec() << " secs. ";

  LOG(INFO) << "The map contains "
            << map_->getTsdfLayerPtr()->getNumberOfAllocatedBlocks()
            << " tsdf and "
            << map_->getLabelLayerPtr()->getNumberOfAllocatedBlocks()
            << " label blocks.";

  start = ros::WallTime::now();

  integrator_->mergeLabels(&merges_to_publish_);
  integrator_->getLabelsToPublish(&segment_labels_to_publish_);

  end = ros::WallTime::now();
  LOG(INFO) << "Merged segments in " << (end - start).toSec() << " seconds.";
  start = ros::WallTime::now();

  segment_merge_candidates_.clear();
  segment_label_candidates.clear();
  for (Segment* segment : segments_to_integrate_) {
    delete segment;
  }
  segments_to_integrate_.clear();

  end = ros::WallTime::now();
  LOG(INFO) << "Cleared candidates and memory in " << (end - start).toSec()
            << " seconds.";

  LOG(INFO) << "Timings: " << std::endl << timing::Timing::Print() << std::endl;
}

void Controller::segmentPointCloudCallback(
    const sensor_msgs::PointCloud2::Ptr& segment_point_cloud_msg) {
  if (!integration_on_) {
    return;
  }
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
    } else {
      LOG(INFO) << "No segments to integrate.";
    }
  }
  received_first_message_ = true;
  last_segment_msg_timestamp_ = segment_point_cloud_msg->header.stamp;

  processSegment(segment_point_cloud_msg);
}

void Controller::resetMeshIntegrators() {
  label_tsdf_mesh_config_.color_scheme =
      MeshLabelIntegrator::ColorScheme::kMerged;

  mesh_merged_integrator_.reset(
      new MeshLabelIntegrator(mesh_config_, label_tsdf_mesh_config_, map_.get(),
                              mesh_merged_layer_.get(), &need_full_remesh_));

  if (multiple_visualizers_) {
    label_tsdf_mesh_config_.color_scheme =
        MeshLabelIntegrator::ColorScheme::kLabel;
    mesh_label_integrator_.reset(new MeshLabelIntegrator(
        mesh_config_, label_tsdf_mesh_config_, map_.get(),
        mesh_label_layer_.get(), &need_full_remesh_));
    label_tsdf_mesh_config_.color_scheme =
        MeshLabelIntegrator::ColorScheme::kSemantic;
    mesh_semantic_integrator_.reset(new MeshLabelIntegrator(
        mesh_config_, label_tsdf_mesh_config_, map_.get(),
        mesh_semantic_layer_.get(), &need_full_remesh_));
    label_tsdf_mesh_config_.color_scheme =
        MeshLabelIntegrator::ColorScheme::kInstance;
    mesh_instance_integrator_.reset(new MeshLabelIntegrator(
        mesh_config_, label_tsdf_mesh_config_, map_.get(),
        mesh_instance_layer_.get(), &need_full_remesh_));
  }
}

// TODO(margaritaG): this is not thread safe wrt segmentPointCloudCallback yet.
bool Controller::resetMapCallback(std_srvs::Empty::Request& /*request*/,
                                  std_srvs::Empty::Response& /*request*/) {
  // Reset counters and flags.
  integrated_frames_count_ = 0u;
  received_first_message_ = false;
  {
    std::lock_guard<std::mutex> label_tsdf_layers_lock(
        label_tsdf_layers_mutex_);

    map_.reset(new LabelTsdfMap(map_config_));
    integrator_.reset(new LabelTsdfIntegrator(
        tsdf_integrator_config_, label_tsdf_integrator_config_, map_.get()));
  }
  // Clear the mesh layers.
  {
    std::lock_guard<std::mutex> mesh_layer_lock(mesh_layer_mutex_);

    mesh_label_layer_->clear();
    mesh_semantic_layer_->clear();
    mesh_instance_layer_->clear();
    mesh_merged_layer_->clear();

    resetMeshIntegrators();
    need_full_remesh_ = true;
  }

  // Clear segments to be integrated from the last frame.
  segment_merge_candidates_.clear();
  segment_label_candidates.clear();
  for (Segment* segment : segments_to_integrate_) {
    delete segment;
  }
  segments_to_integrate_.clear();

  return true;
}

bool Controller::toggleIntegrationCallback(
    std_srvs::SetBool::Request& request,
    std_srvs::SetBool::Response& response) {
  if (request.data ^ integration_on_) {
    integration_on_ = request.data;
    response.success = true;
  } else {
    response.success = false;
    response.message = "Integration is already " +
                       std::string(integration_on_ ? "ON." : "OFF.");
  }

  return true;
}

bool Controller::getMapCallback(vpp_msgs::GetMap::Request& /* request */,
                                vpp_msgs::GetMap::Response& response) {
  pcl::PointCloud<PointMapType> map_pointcloud;

  {
    std::lock_guard<std::mutex> label_tsdf_layers_lock(
        label_tsdf_layers_mutex_);
    createPointcloudFromMap(*map_, &map_pointcloud);
  }

  map_pointcloud.header.frame_id = world_frame_;
  pcl::toROSMsg(map_pointcloud, response.map_cloud);

  response.voxel_size = map_config_.voxel_size;

  if (publish_scene_map_) {
    map_cloud_pub_->publish(map_pointcloud);
  }

  return true;
}

bool Controller::generateMeshCallback(std_srvs::Empty::Request& request,
                                      std_srvs::Empty::Response& response) {
  constexpr bool kClearMesh = true;
  generateMesh(kClearMesh);
  return true;
}

bool Controller::getScenePointcloudCallback(
    vpp_msgs::GetScenePointcloud::Request& /* request */,
    vpp_msgs::GetScenePointcloud::Response& response) {
  pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
  fillPointcloudWithMesh(mesh_merged_layer_, ColorMode::kColor, &pointcloud);

  pointcloud.header.frame_id = world_frame_;
  pcl::toROSMsg(pointcloud, response.scene_cloud);

  scene_cloud_pub_->publish(pointcloud);

  return true;
}

bool Controller::saveSegmentsAsMeshCallback(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  Labels labels;
  std::unordered_map<Label, LabelTsdfMap::LayerPair> label_to_layers;
  {
    std::lock_guard<std::mutex> label_tsdf_layers_lock(
        label_tsdf_layers_mutex_);
    // Get list of all labels in the map.
    labels = map_->getLabelList();

    // Extract the TSDF and label layers corresponding to each segment.
    constexpr bool kLabelsListIsComplete = true;
    map_->extractSegmentLayers(labels, &label_to_layers, kLabelsListIsComplete);
  }

  const char* kSegmentFolder = "gsm_segments";

  struct stat info;
  if (stat(kSegmentFolder, &info) != 0) {
    CHECK_EQ(mkdir(kSegmentFolder, ACCESSPERMS), 0);
  }

  bool overall_success = true;
  for (Label label : labels) {
    auto it = label_to_layers.find(label);
    CHECK(it != label_to_layers.end())
        << "Layers for label " << label << " could not be extracted.";

    const Layer<TsdfVoxel>& segment_tsdf_layer = it->second.first;
    const Layer<LabelVoxel>& segment_label_layer = it->second.second;

    CHECK_EQ(voxblox::file_utils::makePath("gsm_segments", 0777), 0);

    std::string mesh_filename =
        "gsm_segments/gsm_segment_mesh_label_" + std::to_string(label) + ".ply";

    bool success = voxblox::io::outputLayerAsPly(
        segment_tsdf_layer, mesh_filename,
        voxblox::io::PlyOutputTypes::kSdfIsosurface);

    if (success) {
      LOG(INFO) << "Output segment file as PLY: " << mesh_filename.c_str();
    } else {
      LOG(INFO) << "Failed to output mesh as PLY:" << mesh_filename.c_str();
    }
  }

  return overall_success;
}

bool Controller::getListSemanticInstancesCallback(
    vpp_msgs::GetListSemanticInstances::Request& /* request */,
    vpp_msgs::GetListSemanticInstances::Response& response) {
  SemanticLabels semantic_labels;

  {
    std::lock_guard<std::mutex> label_tsdf_layers_lock(
        label_tsdf_layers_mutex_);
    // Get list of instances and their corresponding semantic category.
    map_->getSemanticInstanceList(&response.instance_ids, &semantic_labels);
  }

  // Map class id to human-readable semantic category label.
  for (const SemanticLabel semantic_label : semantic_labels) {
    response.semantic_categories.push_back(classes[(unsigned)semantic_label]);
  }
  return true;
}

bool Controller::getAlignedInstanceBoundingBoxCallback(
    vpp_msgs::GetAlignedInstanceBoundingBox::Request& request,
    vpp_msgs::GetAlignedInstanceBoundingBox::Response& response) {
  InstanceLabels all_instance_labels, instance_labels;
  std::unordered_map<InstanceLabel, LabelTsdfMap::LayerPair>
      instance_label_to_layers;
  InstanceLabel instance_label = request.instance_id;

  {
    std::lock_guard<std::mutex> label_tsdf_layers_lock(
        label_tsdf_layers_mutex_);
    // Get list of all instances in the map.
    all_instance_labels = map_->getInstanceList();
  }
  // Check if queried instance id is in the list of instance ids in the map.
  auto instance_label_it = std::find(all_instance_labels.begin(),
                                     all_instance_labels.end(), instance_label);

  if (instance_label_it == all_instance_labels.end()) {
    LOG(ERROR) << "The queried instance ID does not exist in the map.";
    return false;
  }

  instance_labels.push_back(instance_label);
  bool kSaveSegmentsAsPly = false;
  extractInstanceSegments(instance_labels, kSaveSegmentsAsPly,
                          &instance_label_to_layers);

  auto it = instance_label_to_layers.find(instance_label);
  CHECK(it != instance_label_to_layers.end())
      << "Layers for instance label " << instance_label
      << " could not be extracted.";

  const Layer<TsdfVoxel>& segment_tsdf_layer = it->second.first;

  pcl::PointCloud<pcl::PointSurfel>::Ptr instance_pointcloud(
      new pcl::PointCloud<pcl::PointSurfel>);

  convertVoxelGridToPointCloud(segment_tsdf_layer, mesh_config_,
                               instance_pointcloud.get());

  Eigen::Vector3f bbox_translation;
  Eigen::Quaternionf bbox_quaternion;
  Eigen::Vector3f bbox_size;
  computeAlignedBoundingBox(instance_pointcloud, &bbox_translation,
                            &bbox_quaternion, &bbox_size);

  fillAlignedBoundingBoxMsg(bbox_translation, bbox_quaternion, bbox_size,
                            &response.bbox);

  if (publish_object_bbox_) {
    visualization_msgs::Marker bbox_marker;
    fillBoundingBoxMarkerMsg(world_frame_, instance_label, bbox_translation,
                             bbox_quaternion, bbox_size, &bbox_marker);
    bbox_pub_->publish(bbox_marker);

    geometry_msgs::TransformStamped bbox_tf;
    fillBoundingBoxTfMsg(world_frame_, std::to_string(instance_label),
                         bbox_translation, bbox_quaternion, &bbox_tf);
    tf_broadcaster_.sendTransform(bbox_tf);
  }
  return true;
}

bool Controller::extractInstancesCallback(
    std_srvs::Empty::Request& /*request*/,
    std_srvs::Empty::Response& /*response*/) {
  InstanceLabels instance_labels;
  std::unordered_map<InstanceLabel, LabelTsdfMap::LayerPair>
      instance_label_to_layers;
  {
    std::lock_guard<std::mutex> label_tsdf_layers_lock(
        label_tsdf_layers_mutex_);
    // Get list of all instances in the map.
    instance_labels = map_->getInstanceList();
  }

  bool kSaveSegmentsAsPly = true;
  extractInstanceSegments(instance_labels, kSaveSegmentsAsPly,
                          &instance_label_to_layers);

  return true;
}

void Controller::extractInstanceSegments(
    InstanceLabels instance_labels, bool save_segments_as_ply,
    std::unordered_map<InstanceLabel, LabelTsdfMap::LayerPair>*
        instance_label_to_layers) {
  {
    std::lock_guard<std::mutex> label_tsdf_layers_lock(
        label_tsdf_layers_mutex_);

    // Extract the TSDF and label layers corresponding to each instance segment.
    map_->extractInstanceLayers(instance_labels, instance_label_to_layers);
  }

  for (const InstanceLabel instance_label : instance_labels) {
    auto it = instance_label_to_layers->find(instance_label);
    CHECK(it != instance_label_to_layers->end())
        << "Layers for instance label " << instance_label
        << " could not be extracted.";

    if (save_segments_as_ply) {
      const Layer<TsdfVoxel>& segment_tsdf_layer = it->second.first;
      const Layer<LabelVoxel>& segment_label_layer = it->second.second;

      CHECK_EQ(voxblox::file_utils::makePath("vpp_instances", 0777), 0);

      std::string mesh_filename = "vpp_instances/vpp_instance_segment_label_" +
                                  std::to_string(instance_label) + ".ply";

      bool success = voxblox::io::outputLayerAsPly(
          segment_tsdf_layer, mesh_filename,
          voxblox::io::PlyOutputTypes::kSdfIsosurface);

      if (success) {
        LOG(INFO) << "Output segment file as PLY: " << mesh_filename.c_str();
      } else {
        LOG(INFO) << "Failed to output mesh as PLY: " << mesh_filename.c_str();
      }
    }
  }
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
    time_to_lookup = ros::Time(0);
    LOG(ERROR) << "Using latest TF transform instead of timestamp match.";
    return false;
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

// TODO(ntonci): Generalize this and rename since this one is also publishing
// and saving, not just generating.
void Controller::generateMesh(bool clear_mesh) {  // NOLINT
  {
    std::lock_guard<std::mutex> mesh_layer_lock(mesh_layer_mutex_);
    voxblox::timing::Timer generate_mesh_timer("mesh/generate");
    {
      std::lock_guard<std::mutex> label_tsdf_layers_lock(
          label_tsdf_layers_mutex_);

      bool only_mesh_updated_blocks = true;
      if (clear_mesh) {
        only_mesh_updated_blocks = false;
      }

      constexpr bool clear_updated_flag = true;
      mesh_merged_integrator_->generateMesh(only_mesh_updated_blocks,
                                            clear_updated_flag);

      if (multiple_visualizers_) {
        mesh_label_integrator_->generateMesh(only_mesh_updated_blocks,
                                             clear_updated_flag);
        mesh_semantic_integrator_->generateMesh(only_mesh_updated_blocks,
                                                clear_updated_flag);
        mesh_instance_integrator_->generateMesh(only_mesh_updated_blocks,
                                                clear_updated_flag);
      }
      generate_mesh_timer.Stop();
    }

    mesh_layer_updated_ = true;

    if (publish_scene_mesh_) {
      timing::Timer publish_mesh_timer("mesh/publish");
      voxblox_msgs::Mesh mesh_msg;
      // TODO(margaritaG) : this function cleans up empty meshes, and this seems
      // to trouble the visualizer. Investigate.
      generateVoxbloxMeshMsg(mesh_merged_layer_, ColorMode::kColor, &mesh_msg);
      mesh_msg.header.frame_id = world_frame_;
      scene_mesh_pub_->publish(mesh_msg);
      publish_mesh_timer.Stop();
    }
  }

  if (!mesh_filename_.empty()) {
    timing::Timer output_mesh_timer("mesh/output");
    bool success = outputMeshLayerAsPly("merged_" + mesh_filename_, false,
                                        *mesh_merged_layer_);
    if (multiple_visualizers_) {
      success &= outputMeshLayerAsPly("label_" + mesh_filename_, false,
                                      *mesh_label_layer_);
      success &= outputMeshLayerAsPly("semantic_" + mesh_filename_, false,
                                      *mesh_semantic_layer_);
      success &= outputMeshLayerAsPly("instance_" + mesh_filename_, false,
                                      *mesh_instance_layer_);
    }
    output_mesh_timer.Stop();
    if (success) {
      LOG(INFO) << "Output file as PLY: " << mesh_filename_.c_str();
    } else {
      LOG(INFO) << "Failed to output mesh as PLY: " << mesh_filename_.c_str();
    }
  }

  LOG(INFO) << "Mesh Timings: " << std::endl
            << voxblox::timing::Timing::Print();
}

void Controller::updateMeshEvent(const ros::TimerEvent& e) {
  std::lock_guard<std::mutex> mesh_layer_lock(mesh_layer_mutex_);
  {
    std::lock_guard<std::mutex> label_tsdf_layers_lock(
        label_tsdf_layers_mutex_);
    timing::Timer generate_mesh_timer("mesh/update");
    bool only_mesh_updated_blocks = true;
    if (need_full_remesh_) {
      only_mesh_updated_blocks = false;
      need_full_remesh_ = false;
    }

    if (multiple_visualizers_) {
      bool clear_updated_flag = false;
      mesh_layer_updated_ |= mesh_label_integrator_->generateMesh(
          only_mesh_updated_blocks, clear_updated_flag);
      mesh_layer_updated_ |= mesh_instance_integrator_->generateMesh(
          only_mesh_updated_blocks, clear_updated_flag);
      mesh_layer_updated_ |= mesh_semantic_integrator_->generateMesh(
          only_mesh_updated_blocks, clear_updated_flag);
    }

    bool clear_updated_flag = true;
    // TODO(ntonci): Why not calling generateMesh instead?
    mesh_layer_updated_ |= mesh_merged_integrator_->generateMesh(
        only_mesh_updated_blocks, clear_updated_flag);

    generate_mesh_timer.Stop();
  }

  if (publish_scene_mesh_) {
    timing::Timer publish_mesh_timer("mesh/publish");
    voxblox_msgs::Mesh mesh_msg;
    // TODO(margaritaG) : this function cleans up empty meshes, and this
    // seems to trouble the visualizer. Investigate.
    generateVoxbloxMeshMsg(mesh_merged_layer_, ColorMode::kColor, &mesh_msg);
    mesh_msg.header.frame_id = world_frame_;
    scene_mesh_pub_->publish(mesh_msg);
    publish_mesh_timer.Stop();
  }
}

void Controller::computeAlignedBoundingBox(
    const pcl::PointCloud<pcl::PointSurfel>::Ptr surfel_cloud,
    Eigen::Vector3f* bbox_translation, Eigen::Quaternionf* bbox_quaternion,
    Eigen::Vector3f* bbox_size) {
  CHECK(surfel_cloud);
  CHECK_NOTNULL(bbox_translation);
  CHECK_NOTNULL(bbox_quaternion);
  CHECK_NOTNULL(bbox_size);
#ifdef APPROXMVBB_AVAILABLE
  ApproxMVBB::Matrix3Dyn points(3, surfel_cloud->points.size());

  for (size_t i = 0u; i < surfel_cloud->points.size(); ++i) {
    points(0, i) = double(surfel_cloud->points.at(i).x);
    points(1, i) = double(surfel_cloud->points.at(i).y);
    points(2, i) = double(surfel_cloud->points.at(i).z);
  }

  constexpr double kEpsilon = 0.02;
  constexpr size_t kPointSamples = 200u;
  constexpr size_t kGridSize = 5u;
  constexpr size_t kMvbbDiamOptLoops = 0u;
  constexpr size_t kMvbbGridSearchOptLoops = 0u;
  ApproxMVBB::OOBB oobb =
      ApproxMVBB::approximateMVBB(points, kEpsilon, kPointSamples, kGridSize,
                                  kMvbbDiamOptLoops, kMvbbGridSearchOptLoops);

  ApproxMVBB::Matrix33 A_KI = oobb.m_q_KI.matrix().transpose();
  const int size = points.cols();
  for (unsigned int i = 0u; i < size; ++i) {
    oobb.unite(A_KI * points.col(i));
  }

  constexpr double kExpansionPercentage = 0.05;
  oobb.expandToMinExtentRelative(kExpansionPercentage);

  ApproxMVBB::Vector3 min_in_I = oobb.m_q_KI * oobb.m_minPoint;
  ApproxMVBB::Vector3 max_in_I = oobb.m_q_KI * oobb.m_maxPoint;

  *bbox_quaternion = oobb.m_q_KI.cast<float>();
  *bbox_translation = ((min_in_I + max_in_I) / 2).cast<float>();
  *bbox_size = ((oobb.m_maxPoint - oobb.m_minPoint).cwiseAbs()).cast<float>();
#else
  LOG(WARNING) << "ApproxMVBB is not available and therefore "
                  "bounding box functionality is disabled.";
#endif
}

}  // namespace voxblox_gsm
}  // namespace voxblox
