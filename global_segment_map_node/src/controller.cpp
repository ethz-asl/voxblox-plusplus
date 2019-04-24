// Copyright 2018 Margarita Grinvald, ASL, ETH Zurich, Switzerland

#include "voxblox_gsm/controller.h"

#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <geometry_msgs/TransformStamped.h>
#include <glog/logging.h>
#include <minkindr_conversions/kindr_tf.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <voxblox/core/common.h>
#include <voxblox/integrator/merge_integration.h>
#include <voxblox/utils/layer_utils.h>
#include <voxblox_ros/conversions.h>
#include <voxblox_ros/mesh_vis.h>

#ifdef APPROXMVBB_AVAILABLE
#include <ApproxMVBB/ComputeApproxMVBB.hpp>
#endif

#include <boost/thread.hpp>

#include "voxblox_gsm/conversions.h"
#include "voxblox_gsm/feature_ros_tools.h"

namespace voxblox {
namespace voxblox_gsm {

bool updatedMesh;
boost::mutex updateMeshMutex;

// TODO(grinvalm): make it more efficient by only updating the
// necessary polygons and not all of them each time.
void visualizeMesh(const MeshLayer& mesh_layer) {
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("GSM viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  // viewer->addCoordinateSystem(0.2);
  viewer->initCameraParameters();
  // TODO(grinvalm): find some general default parameters.
  // // 066 position
  // viewer->setCameraPosition(-0.258698, 2.4965, 2.50443, -0.40446, 0.988025,
  //                           0.279138, -0.0487525, 0.828238, -0.558252);
  // viewer->setCameraClipDistances(1.35139, 6.41007);
  // Tango1 position
  viewer->setCameraPosition(0.728046, -1.6836, 2.64582, -0.933718, -0.127688,
                            -0.572224, -0.757564, 0.341492, 0.556309);
  viewer->setCameraClipDistances(0.0106726, 10.6726);

  pcl::PointCloud<pcl::PointXYZRGBA> cloud;
  pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud_ptr(&cloud);

  while (!viewer->wasStopped()) {
    voxblox::Mesh mesh;
    constexpr int updateIntervalms = 1000;
    viewer->spinOnce(updateIntervalms);

    boost::mutex::scoped_lock updatedMeshLock(updateMeshMutex);

    if (updatedMesh) {
      cloud.points.clear();

      mesh_layer.getMesh(&mesh);

      size_t vert_idx = 0;
      for (const Point& vert : mesh.vertices) {
        const Color& color = mesh.colors[vert_idx];
        pcl::PointXYZRGBA point;
        point.r = color.r;
        point.g = color.g;
        point.b = color.b;
        point.a = color.a;
        // = pcl::PointXYZRGBA(color.r, color.g, color.b, color.a);
        point.x = vert(0);
        point.y = vert(1);
        point.z = vert(2);
        cloud.points.push_back(point);

        vert_idx++;
      }

      pcl::PCLPointCloud2 pcl_pc;
      pcl::toPCLPointCloud2(cloud, pcl_pc);
      std::vector<pcl::Vertices> polygons;

      for (size_t i = 0u; i < mesh.indices.size(); i += 3u) {
        pcl::Vertices face;
        for (int j = 0; j < 3; j++) {
          face.vertices.push_back(mesh.indices.at(i + j));
        }
        polygons.push_back(face);
      }

      pcl::PolygonMesh polygon_mesh;
      polygon_mesh.cloud = pcl_pc;
      polygon_mesh.polygons = polygons;
      //
      viewer->removePolygonMesh("meshes");
      if (!viewer->updatePolygonMesh(polygon_mesh, "meshes")) {
        viewer->addPolygonMesh(polygon_mesh, "meshes", 0);
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
      integrated_frames_count_(0u),
      tf_listener_(ros::Duration(100)),
      world_frame_("world"),
      camera_frame_(""),
      no_update_timeout_(0.0),
      publish_gsm_updates_(false),
      publish_scene_mesh_(false),
      received_first_message_(false),
      compute_and_publish_bbox_(false),
      publish_feature_blocks_marker_(false),
      use_label_propagation_(true) {
  CHECK_NOTNULL(node_handle_private_);

  node_handle_private_->param<std::string>("world_frame_id", world_frame_,
                                           world_frame_);
  node_handle_private_->param<std::string>("camera_frame_id", camera_frame_,
                                           camera_frame_);

  // Determine map parameters.
  map_config_.voxel_size = 0.01f;
  map_config_.voxels_per_side = 8u;

  // Workaround for OS X on mac mini not having specializations for float
  // for some reason.
  int voxels_per_side = map_config_.voxels_per_side;
  node_handle_private_->param<FloatingPoint>(
      "voxel_size", map_config_.voxel_size, map_config_.voxel_size);
  node_handle_private_->param<int>("voxels_per_side", voxels_per_side,
                                   voxels_per_side);
  if (!isPowerOfTwo(voxels_per_side)) {
    ROS_ERROR("voxels_per_side must be a power of 2, setting to default value");
    voxels_per_side = map_config_.voxels_per_side;
  }
  map_config_.voxels_per_side = voxels_per_side;

  map_.reset(new LabelTsdfMap(map_config_));
  feature_layer_.reset(new FeatureLayer<Feature3D>(
      map_config_.voxel_size * map_config_.voxels_per_side));

  // Determine TSDF integrator parameters.
  LabelTsdfIntegrator::Config integrator_config;
  integrator_config.voxel_carving_enabled = false;
  integrator_config.allow_clear = true;
  FloatingPoint truncation_distance_factor = 5.0f;
  integrator_config.max_ray_length_m = 2.5f;

  node_handle_private_->param<bool>("voxel_carving_enabled",
                                    integrator_config.voxel_carving_enabled,
                                    integrator_config.voxel_carving_enabled);
  node_handle_private_->param<bool>("allow_clear",
                                    integrator_config.allow_clear,
                                    integrator_config.allow_clear);
  node_handle_private_->param<FloatingPoint>("truncation_distance_factor",
                                             truncation_distance_factor,
                                             truncation_distance_factor);
  node_handle_private_->param<FloatingPoint>(
      "max_ray_length_m", integrator_config.max_ray_length_m,
      integrator_config.max_ray_length_m);

  integrator_config.default_truncation_distance =
      map_config_.voxel_size * truncation_distance_factor;

  std::string method("merged");
  node_handle_private_->param<std::string>("method", method, method);
  if (method.compare("merged") == 0) {
    integrator_config.enable_anti_grazing = false;
  } else if (method.compare("merged_discard") == 0) {
    integrator_config.enable_anti_grazing = true;
  } else {
    integrator_config.enable_anti_grazing = false;
  }

  // Determine label integrator parameters.
  LabelTsdfIntegrator::LabelTsdfConfig label_tsdf_integrator_config;
  label_tsdf_integrator_config.enable_pairwise_confidence_merging = true;
  label_tsdf_integrator_config.object_flushing_age_threshold = 30;

  node_handle_private_->param<bool>(
      "enable_pairwise_confidence_merging",
      label_tsdf_integrator_config.enable_pairwise_confidence_merging,
      label_tsdf_integrator_config.enable_pairwise_confidence_merging);
  node_handle_private_->param<FloatingPoint>(
      "pairwise_confidence_ratio_threshold",
      label_tsdf_integrator_config.pairwise_confidence_ratio_threshold,
      label_tsdf_integrator_config.pairwise_confidence_ratio_threshold);
  node_handle_private_->param<int>(
      "pairwise_confidence_count_threshold",
      label_tsdf_integrator_config.pairwise_confidence_count_threshold,
      label_tsdf_integrator_config.pairwise_confidence_count_threshold);
  node_handle_private_->param<int>(
      "object_flushing_age_threshold",
      label_tsdf_integrator_config.object_flushing_age_threshold,
      label_tsdf_integrator_config.object_flushing_age_threshold);

  integrator_.reset(new LabelTsdfIntegrator(
      integrator_config, label_tsdf_integrator_config, map_->getTsdfLayerPtr(),
      map_->getLabelLayerPtr(), map_->getHighestLabelPtr()));

  // Determine feature integrator parameters.
  // TODO(ntonci): Make rosparam if there are any.
  FeatureIntegrator::Config feature_integrator_config;
  feature_integrator_.reset(
      new FeatureIntegrator(feature_integrator_config, feature_layer_.get()));

  mesh_layer_.reset(new MeshLayer(map_->block_size()));
  mesh_integrator_.reset(new MeshLabelIntegrator(
      mesh_config_, map_->getTsdfLayerPtr(), map_->getLabelLayerPtr(),
      mesh_layer_.get(), *integrator_->getLabelsAgeMapPtr()));

  // Visualization settings.
  bool visualize = false;
  node_handle_private_->param<bool>("visualize", visualize, visualize);
  if (visualize) {
    boost::thread visualizerThread(visualizeMesh, boost::ref(*mesh_layer_));
  }

  node_handle_private_->param<bool>(
      "publish_segment_mesh", publish_segment_mesh_, publish_segment_mesh_);
  node_handle_private_->param<bool>("publish_scene_mesh", publish_scene_mesh_,
                                    publish_scene_mesh_);
  node_handle_private_->param<bool>("compute_and_publish_bbox",
                                    compute_and_publish_bbox_,
                                    compute_and_publish_bbox_);
  node_handle_private_->param<bool>("publish_feature_blocks_marker",
                                    publish_feature_blocks_marker_,
                                    publish_feature_blocks_marker_);
  node_handle_private_->param<bool>(
      "use_label_propagation", use_label_propagation_, use_label_propagation_);

#ifndef APPROXMVBB_AVAILABLE
  if (compute_and_publish_bbox_) {
    ROS_WARN_STREAM(
        "ApproxMVBB is not available and therefore bounding box functionality "
        "is disabled.");
  }
  compute_and_publish_bbox_ = false;
#endif

  // If set, use a timer to progressively update the mesh.
  double update_mesh_every_n_sec = 0.0;
  node_handle_private_->param<double>("update_mesh_every_n_sec",
                                      update_mesh_every_n_sec,
                                      update_mesh_every_n_sec);

  if (update_mesh_every_n_sec > 0.0) {
    update_mesh_timer_ = node_handle_private_->createTimer(
        ros::Duration(update_mesh_every_n_sec), &Controller::updateMeshEvent,
        this);
  }

  node_handle_private_->param<std::string>("mesh_filename", mesh_filename_,
                                           mesh_filename_);

  node_handle_private_->param<bool>("publish_gsm_updates", publish_gsm_updates_,
                                    publish_gsm_updates_);

  node_handle_private_->param<double>("no_update_timeout", no_update_timeout_,
                                      no_update_timeout_);

  ros::spinOnce();
}

Controller::~Controller() {}

// TODO(ntonci): Move all sub, pub, etc., and helpers to inline header.
void Controller::subscribeFeatureTopic(ros::Subscriber* feature_sub) {
  CHECK_NOTNULL(feature_sub);
  std::string feature_topic = "/rgbd_feature_node/features";
  node_handle_private_->param<std::string>("feature_topic", feature_topic,
                                           feature_topic);

  // Large queue size to give slack to the
  // pipeline and not lose any messages.
  constexpr int kFeatureQueueSize = 2000;
  *feature_sub = node_handle_private_->subscribe(
      feature_topic, kFeatureQueueSize, &Controller::featureCallback, this);
}

void Controller::advertiseFeatureBlockTopic(ros::Publisher* feature_block_pub) {
  CHECK_NOTNULL(feature_block_pub);
  std::string feature_block_topic = "feature_block_array";
  node_handle_private_->param<std::string>(
      "feature_block_topic", feature_block_topic, feature_block_topic);

  constexpr int kFeatureBlockQueueSize = 2000;
  *feature_block_pub =
      node_handle_private_->advertise<visualization_msgs::MarkerArray>(
          feature_block_topic, kFeatureBlockQueueSize, true);
  feature_block_pub_ = feature_block_pub;
}

void Controller::subscribeSegmentPointCloudTopic(
    ros::Subscriber* segment_point_cloud_sub) {
  CHECK_NOTNULL(segment_point_cloud_sub);
  std::string segment_point_cloud_topic =
      "/depth_segmentation_node/object_segment";
  node_handle_private_->param<std::string>("segment_point_cloud_topic",
                                           segment_point_cloud_topic,
                                           segment_point_cloud_topic);

  // Large queue size to give slack to the
  // pipeline and not lose any messages.
  constexpr int kSegmentPointCloudQueueSize = 2000;
  *segment_point_cloud_sub = node_handle_private_->subscribe(
      segment_point_cloud_topic, kSegmentPointCloudQueueSize,
      &Controller::segmentPointCloudCallback, this);
}

void Controller::advertiseSegmentGsmUpdateTopic(
    ros::Publisher* segment_gsm_update_pub) {
  CHECK_NOTNULL(segment_gsm_update_pub);
  std::string segment_gsm_update_topic = "gsm_update";
  node_handle_private_->param<std::string>("segment_gsm_update_topic",
                                           segment_gsm_update_topic,
                                           segment_gsm_update_topic);
  // TODO(ff): Reduce this value, once we know some reasonable limit.
  constexpr int kGsmUpdateQueueSize = 2000;
  *segment_gsm_update_pub =
      node_handle_private_->advertise<modelify_msgs::GsmUpdate>(
          segment_gsm_update_topic, kGsmUpdateQueueSize, true);
  segment_gsm_update_pub_ = segment_gsm_update_pub;
}

void Controller::advertiseSceneGsmUpdateTopic(
    ros::Publisher* scene_gsm_update_pub) {
  CHECK_NOTNULL(scene_gsm_update_pub);
  std::string scene_gsm_update_topic = "scene";
  node_handle_private_->param<std::string>(
      "scene_gsm_update_topic", scene_gsm_update_topic, scene_gsm_update_topic);
  constexpr int kGsmSceneQueueSize = 1;

  *scene_gsm_update_pub =
      node_handle_private_->advertise<modelify_msgs::GsmUpdate>(
          scene_gsm_update_topic, kGsmSceneQueueSize, true);
  scene_gsm_update_pub_ = scene_gsm_update_pub;
}

void Controller::advertiseSegmentMeshTopic(ros::Publisher* segment_mesh_pub) {
  CHECK_NOTNULL(segment_mesh_pub);
  *segment_mesh_pub = node_handle_private_->advertise<voxblox_msgs::Mesh>(
      "segment_mesh", 1, true);

  segment_mesh_pub_ = segment_mesh_pub;
}

void Controller::advertiseSceneMeshTopic(ros::Publisher* scene_mesh_pub) {
  CHECK_NOTNULL(scene_mesh_pub);
  *scene_mesh_pub =
      node_handle_private_->advertise<voxblox_msgs::Mesh>("mesh", 1, true);

  scene_mesh_pub_ = scene_mesh_pub;
}

void Controller::advertiseBboxTopic(ros::Publisher* bbox_pub) {
  CHECK_NOTNULL(bbox_pub);
  *bbox_pub = node_handle_private_->advertise<visualization_msgs::Marker>(
      "bbox", 1, true);

  bbox_pub_ = bbox_pub;
}

void Controller::advertisePublishSceneService(
    ros::ServiceServer* publish_scene_srv) {
  CHECK_NOTNULL(publish_scene_srv);
  static const std::string kAdvertisePublishSceneServiceName = "publish_scene";
  *publish_scene_srv = node_handle_private_->advertiseService(
      kAdvertisePublishSceneServiceName, &Controller::publishSceneCallback,
      this);
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

void Controller::advertiseSaveSegmentsAsMeshService(
    ros::ServiceServer* save_segments_as_mesh_srv) {
  CHECK_NOTNULL(save_segments_as_mesh_srv);
  *save_segments_as_mesh_srv = node_handle_private_->advertiseService(
      "save_segments_as_mesh", &Controller::saveSegmentsAsMeshCallback, this);
}

template <>
void Controller::fillSegmentWithData<PointSurfelLabel>(
    const sensor_msgs::PointCloud2::Ptr& segment_point_cloud_msg,
    Segment* segment) {
  pcl::PointCloud<PointSurfelLabel> point_cloud;

  pcl::fromROSMsg(*segment_point_cloud_msg, point_cloud);

  segment->points_C_.reserve(point_cloud.points.size());
  segment->colors_.reserve(point_cloud.points.size());

  for (size_t i = 0u; i < point_cloud.points.size(); ++i) {
    if (!std::isfinite(point_cloud.points[i].x) ||
        !std::isfinite(point_cloud.points[i].y) ||
        !std::isfinite(point_cloud.points[i].z)) {
      continue;
    }

    segment->points_C_.push_back(Point(point_cloud.points[i].x,
                                       point_cloud.points[i].y,
                                       point_cloud.points[i].z));

    segment->colors_.push_back(
        Color(point_cloud.points[i].r, point_cloud.points[i].g,
              point_cloud.points[i].b, point_cloud.points[i].a));

    segment->labels_.push_back(point_cloud.points[i].label);
  }
}

void Controller::fromFeaturesMsgToFeature3D(
    const modelify_msgs::Features& features_msg, size_t* number_of_features,
    std::string* camera_frame, ros::Time* timestamp,
    std::vector<Feature3D>* features_C) {
  CHECK_NOTNULL(camera_frame);
  CHECK_NOTNULL(timestamp);
  CHECK_NOTNULL(features_C);

  *number_of_features = features_msg.length;
  *camera_frame = features_msg.header.frame_id;
  *timestamp = features_msg.header.stamp;

  for (const modelify_msgs::Feature& msg : features_msg.features) {
    Feature3D feature;

    feature.keypoint << msg.x, msg.y, msg.z;
    feature.keypoint_scale = msg.scale;
    feature.keypoint_response = msg.response;
    feature.keypoint_angle = msg.angle;

    // TODO(ntonci): This should depend on the descriptor, currently it assumes
    // SIFT. Template!
    constexpr size_t kRows = 1;
    constexpr size_t kCols = 128;
    feature.descriptor = cv::Mat(kRows, kCols, CV_64FC1);
    memcpy(feature.descriptor.data, msg.descriptor.data(),
           msg.descriptor.size() * sizeof(double));

    CHECK_EQ(feature.descriptor.cols, msg.descriptor.size())
        << "Descriptor size is wrong!";

    features_C->push_back(feature);
  }

  if (features_C->size() != *number_of_features) {
    ROS_WARN_STREAM(
        "Number of features is different from the one "
        "specified in the message: "
        << features_C->size() << " vs. " << *number_of_features);
  }
}

void Controller::featureCallback(const modelify_msgs::Features& features_msg) {
  size_t number_of_features;
  std::string camera_frame;
  ros::Time timestamp;
  std::vector<Feature3D> features_C;
  fromFeaturesMsgToFeature3D(features_msg, &number_of_features, &camera_frame,
                             &timestamp, &features_C);

  if (camera_frame != camera_frame_) {
    ROS_WARN_STREAM("Camera frame in the header ("
                    << camera_frame
                    << "), is not the same as specified rosparam camera frame ("
                    << camera_frame_ << ").");
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

    if (use_label_propagation_) {
      ros::WallTime start = ros::WallTime::now();

      integrator_->decideLabelPointClouds(&segments_to_integrate_,
                                          &segment_label_candidates,
                                          &segment_merge_candidates_);

      ros::WallTime end = ros::WallTime::now();
      ROS_INFO("Decided labels for %lu pointclouds in %f seconds.",
               segments_to_integrate_.size(), (end - start).toSec());
    }

    constexpr bool kIsFreespacePointcloud = false;

    ros::WallTime start = ros::WallTime::now();

    for (const auto& segment : segments_to_integrate_) {
      integrator_->integratePointCloud(segment->T_G_C_, segment->points_C_,
                                       segment->colors_, segment->labels_,
                                       kIsFreespacePointcloud);
    }

    ros::WallTime end = ros::WallTime::now();
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

    if (publish_gsm_updates_ && publishObjects()) {
      publishScene();
    }
  }
  received_first_message_ = true;
  last_update_received_ = ros::Time::now();
  last_segment_msg_timestamp_ = segment_point_cloud_msg->header.stamp;

  // Look up transform from camera frame to world frame.
  Transformation T_G_C;
  if (lookupTransform(camera_frame_, world_frame_,
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

    if (use_label_propagation_) {
      fillSegmentWithData(segment_point_cloud_msg, segment);
    } else {
      fillSegmentWithData<PointSurfelLabel>(segment_point_cloud_msg, segment);
    }

    segment->T_G_C_ = T_G_C;

    if (use_label_propagation_) {
      ros::WallTime start = ros::WallTime::now();
      integrator_->computeSegmentLabelCandidates(
          segment, &segment_label_candidates, &segment_merge_candidates_);

      ros::WallTime end = ros::WallTime::now();
      ROS_INFO(
          "Computed label candidates for a pointcloud of size %lu in %f "
          "seconds.",
          segment->points_C_.size(), (end - start).toSec());
    }

    ROS_INFO_STREAM("Timings: " << std::endl << timing::Timing::Print());
  }
}

bool Controller::publishSceneCallback(std_srvs::SetBool::Request& request,
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

  evaluateLayerRmseAtPoses<TsdfVoxelType>(
      voxel_evaluation_mode, map_->getTsdfLayer(),
      *(merged_object_layer_O.get()), transforms_W_O,
      &voxel_evaluation_details_vector);

  voxblox_gsm::voxelEvaluationDetails2VoxelEvaluationDetailsMsg(
      voxel_evaluation_details_vector, &response.voxel_evaluation_details);
  return true;
}

bool Controller::generateMeshCallback(
    std_srvs::Empty::Request& request,
    std_srvs::Empty::Response& response) {  // NOLINT
  constexpr bool kClearMesh = true;
  generateMesh(kClearMesh);
  return true;
}

bool Controller::saveSegmentsAsMeshCallback(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  // Get list of all labels in the map.
  std::vector<Label> labels = integrator_->getLabelsList();
  static constexpr bool kConnectedMesh = false;

  std::unordered_map<Label, LayerTuple> label_to_layers;
  // Extract the TSDF and label layers corresponding to a segment.
  constexpr bool kLabelsListIsComplete = true;
  extractSegmentLayers(labels, &label_to_layers, kLabelsListIsComplete);

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

    const Layer<TsdfVoxel>& segment_tsdf_layer =
        std::get<LayerAccessor::kTsdfLayer>(it->second);
    const Layer<LabelVoxel>& segment_label_layer =
        std::get<LayerAccessor::kLabelLayer>(it->second);

    voxblox::Mesh segment_mesh;
    if (convertTsdfLabelLayersToMesh(segment_tsdf_layer, segment_label_layer,
                                     &segment_mesh, kConnectedMesh)) {
      std::string mesh_filename = std::string(kSegmentFolder) +
                                  "/gsm_segment_mesh_label_" +
                                  std::to_string(label) + ".ply";

      bool success = outputMeshAsPly(mesh_filename, segment_mesh);
      if (success) {
        ROS_INFO("Output segment file as PLY: %s", mesh_filename.c_str());
      } else {
        ROS_INFO("Failed to output mesh as PLY: %s", mesh_filename.c_str());
      }
      overall_success &= success;
    }
  }

  return overall_success;
}

void Controller::extractSegmentLayers(
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
  FeatureLayer<Feature3D> feature_layer_empty(map_config_.voxel_size *
                                              map_config_.voxels_per_side);

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

// TODO(ff): Create this somewhere:
// void serializeGsmAsMsg(const map&, const label&, const parent_labels&, msg*);
bool Controller::publishObjects(const bool publish_all) {
  CHECK_NOTNULL(segment_gsm_update_pub_);
  bool published_segment_label = false;
  std::vector<Label> labels_to_publish;
  getLabelsToPublish(publish_all, &labels_to_publish);

  std::unordered_map<Label, LayerTuple> label_to_layers;
  ros::Time start = ros::Time::now();
  extractSegmentLayers(labels_to_publish, &label_to_layers, publish_all);
  ros::Time stop = ros::Time::now();
  ros::Duration duration = stop - start;
  LOG(INFO) << "Extracting segment layers took " << duration.toSec() << "s";

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
    MeshIntegratorConfig mesh_config;
    node_handle_private_->param<float>("mesh_config/min_weight",
                                       mesh_config.min_weight,
                                       mesh_config.min_weight);
    pcl::PointCloud<pcl::PointSurfel>::Ptr surfel_cloud(
        new pcl::PointCloud<pcl::PointSurfel>());
    convertVoxelGridToPointCloud(tsdf_layer, mesh_config, surfel_cloud.get());

    if (surfel_cloud->empty()) {
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
                                      &gsm_update_msg.object.feature_layer,
                                      DeserializeAction::kUpdate);

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
    gsm_update_msg.object.surfel_cloud.header.frame_id = world_frame_;
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
      MeshLabelIntegrator mesh_integrator(mesh_config_, &tsdf_layer,
                                          &label_layer, mesh_layer.get());
      constexpr bool only_mesh_updated_blocks = false;
      constexpr bool clear_updated_flag = true;
      mesh_integrator.generateMesh(only_mesh_updated_blocks,
                                   clear_updated_flag);

      voxblox_msgs::Mesh segment_mesh_msg;
      generateVoxbloxMeshMsg(mesh_layer, ColorMode::kColor, &segment_mesh_msg);
      segment_mesh_msg.header.frame_id = world_frame_;
      segment_mesh_pub_->publish(segment_mesh_msg);
    }
    all_published_segments_.insert(label);
    published_segment_label = true;
  }
  segment_labels_to_publish_.clear();
  return published_segment_label;
}

void Controller::publishScene() {
  CHECK_NOTNULL(scene_gsm_update_pub_);
  modelify_msgs::GsmUpdate gsm_update_msg;

  // TODO(ntonci): Create a method that generalizes this part such that you dont
  // need to do it twice, for objects and scene.
  gsm_update_msg.header.stamp = last_segment_msg_timestamp_;
  gsm_update_msg.header.frame_id = world_frame_;

  constexpr bool kSerializeOnlyUpdated = false;
  serializeLayerAsMsg<TsdfVoxel>(map_->getTsdfLayer(), kSerializeOnlyUpdated,
                                 &gsm_update_msg.object.tsdf_layer);
  // TODO(ff): Make sure this works also, there is no LabelVoxel in voxblox
  // yet, hence it doesn't work.
  // TODO(ntonci): This seems to work?
  serializeLayerAsMsg<LabelVoxel>(map_->getLabelLayer(), kSerializeOnlyUpdated,
                                  &gsm_update_msg.object.label_layer);
  // TODO(ntonci): Also publish feature layer for scene.

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

// TODO(ntonci): Generalize this and rename since this one is also publishing
// and saving, not just generating.
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

  updatedMesh = true;
  updateMeshLock.unlock();

  if (publish_scene_mesh_) {
    timing::Timer publish_mesh_timer("mesh/publish");
    voxblox_msgs::Mesh mesh_msg;
    generateVoxbloxMeshMsg(mesh_layer_, ColorMode::kColor, &mesh_msg);
    mesh_msg.header.frame_id = world_frame_;
    scene_mesh_pub_->publish(mesh_msg);
    publish_mesh_timer.Stop();
  }

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

  ROS_INFO_STREAM("Mesh Timings: " << std::endl
                                   << voxblox::timing::Timing::Print());
}

void Controller::updateMeshEvent(const ros::TimerEvent& e) {
  boost::mutex::scoped_lock updateMeshLock(updateMeshMutex);

  timing::Timer generate_mesh_timer("mesh/update");
  constexpr bool only_mesh_updated_blocks = true;
  constexpr bool clear_updated_flag = true;

  // TODO(ntonci): Why not calling generateMesh instead?
  updatedMesh = mesh_integrator_->generateMesh(only_mesh_updated_blocks,
                                               clear_updated_flag);
  updateMeshLock.unlock();

  generate_mesh_timer.Stop();

  if (publish_scene_mesh_) {
    timing::Timer publish_mesh_timer("mesh/publish");
    voxblox_msgs::Mesh mesh_msg;
    generateVoxbloxMeshMsg(mesh_layer_, ColorMode::kColor, &mesh_msg);
    mesh_msg.header.frame_id = world_frame_;
    scene_mesh_pub_->publish(mesh_msg);
    publish_mesh_timer.Stop();
  }
}

bool Controller::noNewUpdatesReceived() const {
  if (received_first_message_ && no_update_timeout_ != 0.0) {
    return (ros::Time::now() - last_update_received_).toSec() >
           no_update_timeout_;
  }
  return false;
}

void Controller::publishGsmUpdate(const ros::Publisher& publisher,
                                  modelify_msgs::GsmUpdate* gsm_update) {
  publisher.publish(*gsm_update);
}

void Controller::getLabelsToPublish(const bool get_all,
                                    std::vector<Label>* labels) {
  CHECK_NOTNULL(labels);
  if (get_all) {
    *labels = integrator_->getLabelsList();
    ROS_INFO("Publishing all segments");
  } else {
    *labels = segment_labels_to_publish_;
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
  for (unsigned int i = 0; i < size; ++i) {
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
  ROS_WARN_STREAM(
      "Bounding box computation is not supported since ApproxMVBB is "
      "disabled.");
#endif
}

}  // namespace voxblox_gsm
}  // namespace voxblox
