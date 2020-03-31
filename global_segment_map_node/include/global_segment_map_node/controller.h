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
#include <vpp_msgs/GetAlignedInstanceBoundingBox.h>
#include <vpp_msgs/GetListSemanticInstances.h>
#include <vpp_msgs/GetMap.h>
#include <vpp_msgs/GetScenePointcloud.h>

namespace voxblox {
namespace voxblox_gsm {

typedef std::pair<Layer<TsdfVoxel>, Layer<LabelVoxel>> LayerPair;

class Controller {
 public:
  Controller(ros::NodeHandle* node_handle);

  virtual ~Controller();

  void subscribeSegmentPointCloudTopic(
      ros::Subscriber* segment_point_cloud_sub);

  void advertiseMapTopic();

  void advertiseSceneMeshTopic();

  void advertiseSceneCloudTopic();

  void advertiseBboxTopic();

  void advertiseResetMapService(ros::ServiceServer* reset_map_srv);

  void advertiseToggleIntegrationService(
      ros::ServiceServer* toggle_integration_srv);

  void advertiseGetMapService(ros::ServiceServer* get_map_srv);

  void advertiseGenerateMeshService(ros::ServiceServer* generate_mesh_srv);

  void advertiseGetScenePointcloudService(
      ros::ServiceServer* get_scene_pointcloud);

  void advertiseSaveSegmentsAsMeshService(
      ros::ServiceServer* save_segments_as_mesh_srv);

  void advertiseExtractInstancesService(
      ros::ServiceServer* extract_instances_srv);

  void advertiseGetListSemanticInstancesService(
      ros::ServiceServer* get_list_semantic_categories_srv);

  void advertiseGetAlignedInstanceBoundingBoxService(
      ros::ServiceServer* get_instance_bounding_box_srv);

  bool enable_semantic_instance_segmentation_;

  bool publish_scene_map_;
  bool publish_scene_mesh_;
  bool publish_object_bbox_;

  bool use_label_propagation_;

 protected:
  void processSegment(
      const sensor_msgs::PointCloud2::Ptr& segment_point_cloud_msg);

  void integrateFrame(ros::Time msg_timestamp);

  bool resetMapCallback(std_srvs::Empty::Request& request,
                        std_srvs::Empty::Response& response);

  bool toggleIntegrationCallback(std_srvs::SetBool::Request& request,
                                 std_srvs::SetBool::Response& response);

  virtual void segmentPointCloudCallback(
      const sensor_msgs::PointCloud2::Ptr& segment_point_cloud_msg);

  bool getMapCallback(vpp_msgs::GetMap::Request& /* request */,
                      vpp_msgs::GetMap::Response& response);

  bool generateMeshCallback(std_srvs::Empty::Request& request,
                            std_srvs::Empty::Response& response);

  bool getScenePointcloudCallback(
      vpp_msgs::GetScenePointcloud::Request& /* request */,
      vpp_msgs::GetScenePointcloud::Response& response);

  bool saveSegmentsAsMeshCallback(std_srvs::Empty::Request& request,
                                  std_srvs::Empty::Response& response);

  bool extractInstancesCallback(std_srvs::Empty::Request& request,
                                std_srvs::Empty::Response& response);

  bool getListSemanticInstancesCallback(
      vpp_msgs::GetListSemanticInstances::Request& /* request */,
      vpp_msgs::GetListSemanticInstances::Response& response);

  bool getAlignedInstanceBoundingBoxCallback(
      vpp_msgs::GetAlignedInstanceBoundingBox::Request& request,
      vpp_msgs::GetAlignedInstanceBoundingBox::Response& response);

  bool lookupTransform(const std::string& from_frame,
                       const std::string& to_frame, const ros::Time& timestamp,
                       Transformation* transform);

  void generateMesh(bool clear_mesh);

  void updateMeshEvent(const ros::TimerEvent& e);

  // NOT thread safe.
  void resetMeshIntegrators();

  void computeAlignedBoundingBox(
      const pcl::PointCloud<pcl::PointSurfel>::Ptr surfel_cloud,
      Eigen::Vector3f* bbox_translation, Eigen::Quaternionf* bbox_quaternion,
      Eigen::Vector3f* bbox_size);

  void extractInstanceSegments(
      InstanceLabels instance_labels, bool save_segments_as_ply,
      std::unordered_map<InstanceLabel, LabelTsdfMap::LayerPair>*
          instance_label_to_layers);

  ros::NodeHandle* node_handle_private_;

  tf::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  ros::Time last_segment_msg_timestamp_;
  size_t integrated_frames_count_;

  std::string world_frame_;

  bool integration_on_;
  bool received_first_message_;

  LabelTsdfMap::Config map_config_;
  LabelTsdfIntegrator::Config tsdf_integrator_config_;
  LabelTsdfIntegrator::LabelTsdfConfig label_tsdf_integrator_config_;

  std::shared_ptr<LabelTsdfMap> map_;
  std::shared_ptr<LabelTsdfIntegrator> integrator_;

  MeshIntegratorConfig mesh_config_;
  MeshLabelIntegrator::LabelTsdfConfig label_tsdf_mesh_config_;
  ros::Timer update_mesh_timer_;

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
  std::map<Label, std::map<SemanticLabel, int>>* label_class_count_ptr_;

  // Current frame label propagation.
  std::vector<Segment*> segments_to_integrate_;
  std::map<Label, std::map<Segment*, size_t>> segment_label_candidates;
  std::map<Segment*, std::vector<Label>> segment_merge_candidates_;

  // Publishers.
  ros::Publisher* map_cloud_pub_;
  ros::Publisher* scene_mesh_pub_;
  ros::Publisher* scene_cloud_pub_;
  ros::Publisher* bbox_pub_;

  std::thread viz_thread_;
  Visualizer* visualizer_;
  std::mutex label_tsdf_layers_mutex_;
  std::mutex mesh_layer_mutex_;
  bool mesh_layer_updated_;
  bool need_full_remesh_;
  bool multiple_visualizers_;
};

}  // namespace voxblox_gsm
}  // namespace voxblox

#endif  // VOXBLOX_GSM_CONTROLLER_H_
