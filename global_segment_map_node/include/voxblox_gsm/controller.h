// Copyright (c) 2019, ASL, ETH Zurich, Switzerland
// Licensed under the BSD 3-Clause License (see LICENSE for details)

#ifndef VOXBLOX_GSM_CONTROLLER_H_
#define VOXBLOX_GSM_CONTROLLER_H_

#include <vector>

#include <geometry_msgs/Transform.h>

#include <global_segment_map/label_tsdf_integrator.h>
#include <global_segment_map/label_tsdf_map.h>
#include <global_segment_map/label_voxel.h>
#include <global_segment_map/meshing/label_tsdf_mesh_integrator.h>
#include <global_segment_map/utils/visualizer.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox_ros/conversions.h>

namespace voxblox {
namespace voxblox_gsm {

typedef std::pair<Layer<TsdfVoxel>, Layer<LabelVoxel>> LayerPair;

class Controller {
 public:
  Controller(ros::NodeHandle* node_handle);

  virtual ~Controller();

  void subscribeSegmentPointCloudTopic(
      ros::Subscriber* segment_point_cloud_sub);

  void advertiseSceneMeshTopic();

  void advertiseBboxTopic();

  void advertiseGenerateMeshService(ros::ServiceServer* generate_mesh_srv);

  void advertiseSaveSegmentsAsMeshService(
      ros::ServiceServer* save_segments_as_mesh_srv);

  void advertiseExtractInstancesService(
      ros::ServiceServer* extract_instances_srv);

  bool enable_semantic_instance_segmentation_;

  bool publish_scene_mesh_;
  bool compute_and_publish_bbox_;

  bool use_label_propagation_;

 protected:
  void processSegment(
      const sensor_msgs::PointCloud2::Ptr& segment_point_cloud_msg);

  void integrateFrame(ros::Time msg_timestamp);

  virtual void segmentPointCloudCallback(
      const sensor_msgs::PointCloud2::Ptr& segment_point_cloud_msg);

  bool generateMeshCallback(std_srvs::Empty::Request& request,
                            std_srvs::Empty::Response& response);

  bool saveSegmentsAsMeshCallback(std_srvs::Empty::Request& request,
                                  std_srvs::Empty::Response& response);

  bool extractInstancesCallback(std_srvs::Empty::Request& request,
                                std_srvs::Empty::Response& response);

  bool lookupTransform(const std::string& from_frame,
                       const std::string& to_frame, const ros::Time& timestamp,
                       Transformation* transform);

  void generateMesh(bool clear_mesh);

  void updateMeshEvent(const ros::TimerEvent& e);

  void computeAlignedBoundingBox(
      const pcl::PointCloud<pcl::PointSurfel>::Ptr surfel_cloud,
      Eigen::Vector3f* bbox_translation, Eigen::Quaternionf* bbox_quaternion,
      Eigen::Vector3f* bbox_size);

  ros::NodeHandle* node_handle_private_;

  tf::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  ros::Time last_segment_msg_timestamp_;
  size_t integrated_frames_count_;

  std::string world_frame_;

  bool received_first_message_;

  LabelTsdfMap::Config map_config_;
  LabelTsdfIntegrator::Config tsdf_integrator_config_;
  LabelTsdfIntegrator::LabelTsdfConfig label_tsdf_integrator_config_;

  std::shared_ptr<LabelTsdfMap> map_;
  std::shared_ptr<LabelTsdfIntegrator> integrator_;

  MeshIntegratorConfig mesh_config_;
  MeshLabelIntegrator::LabelTsdfConfig label_tsdf_mesh_config_;
  ros::Timer update_mesh_timer_;
  ros::Publisher* scene_mesh_pub_;
  MeshLabelIntegrator::ColorScheme mesh_color_scheme_;
  std::string mesh_filename_;

  std::shared_ptr<MeshLayer> mesh_label_layer_;
  std::shared_ptr<MeshLayer> mesh_semantic_layer_;
  std::shared_ptr<MeshLayer> mesh_instance_layer_;
  std::shared_ptr<MeshLayer> mesh_merged_layer_;
  std::shared_ptr<MeshLabelIntegrator> mesh_label_integrator_;
  std::shared_ptr<MeshLabelIntegrator> mesh_semantic_integrator_;
  std::shared_ptr<MeshLabelIntegrator> mesh_instance_integrator_;
  std::shared_ptr<MeshLabelIntegrator> mesh_merged_integrator_;

  std::vector<Label> segment_labels_to_publish_;
  std::map<Label, std::set<Label>> merges_to_publish_;

  // Semantic labels.
  std::set<SemanticLabel> all_semantic_labels_;
  std::map<Label, std::map<SemanticLabel, int>>* label_class_count_ptr_;

  // Current frame label propagation.
  std::vector<Segment*> segments_to_integrate_;
  std::map<Label, std::map<Segment*, size_t>> segment_label_candidates;
  std::map<Segment*, std::vector<Label>> segment_merge_candidates_;

  ros::Publisher* bbox_pub_;

  std::thread viz_thread_;
  Visualizer* visualizer_;
  std::mutex label_tsdf_layers_mutex_;
  std::mutex mesh_layer_mutex_;
  bool mesh_layer_updated_;
  bool need_full_remesh_;
};

}  // namespace voxblox_gsm
}  // namespace voxblox

#endif  // VOXBLOX_GSM_CONTROLLER_H_
