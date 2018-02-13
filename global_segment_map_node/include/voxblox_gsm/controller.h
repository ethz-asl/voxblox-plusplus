// Copyright 2017 Margarita Grinvald, ASL, ETH Zurich, Switzerland

#ifndef VOXBLOX_GSM_INCLUDE_VOXBLOX_GSM_CONTROLLER_H_
#define VOXBLOX_GSM_INCLUDE_VOXBLOX_GSM_CONTROLLER_H_

#include <vector>

#include <geometry_msgs/Transform.h>
#include <global_segment_map/label_tsdf_integrator.h>
#include <global_segment_map/label_tsdf_map.h>
#include <global_segment_map/label_tsdf_mesh_integrator.h>
#include <modelify_msgs/GsmUpdate.h>
#include <modelify_msgs/ValidateMergedObject.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox_ros/conversions.h>

namespace voxblox {
namespace voxblox_gsm {

class Controller {
 public:
  Controller(ros::NodeHandle* node_handle);

  ~Controller();

  void subscribeSegmentPointCloudTopic(
      ros::Subscriber* segment_point_cloud_sub);

  void advertiseMeshTopic(ros::Publisher* mesh_pub);

  void advertiseSceneTopic(ros::Publisher* scene_pub);

  void advertiseObjectTopic(ros::Publisher* object_pub);

  void advertiseGsmUpdateTopic(ros::Publisher* gsm_update_pub);

  void validateMergedObjectService(
      ros::ServiceServer* validate_merged_object_srv);

  void advertiseGenerateMeshService(ros::ServiceServer* generate_mesh_srv);

  void advertisePublishSceneService(ros::ServiceServer* publish_scene_srv);

  void advertiseExtractSegmentsService(
      ros::ServiceServer* extract_segments_srv);

  void updateMeshEvent(const ros::TimerEvent& e);

  void publishScene();

  void publishObjects(const bool publish_all = false);

  bool noNewUpdatesReceived(const double no_update_timeout) const;

 private:
  void segmentPointCloudCallback(
      const sensor_msgs::PointCloud2::Ptr& segment_point_cloud_msg);

  bool generateMeshCallback(std_srvs::Empty::Request& request,
                            std_srvs::Empty::Response& response);

  bool publishSceneCallback(std_srvs::Empty::Request& request,
                            std_srvs::Empty::Response& response);

  bool validateMergedObjectCallback(
      modelify_msgs::ValidateMergedObject::Request& request,
      modelify_msgs::ValidateMergedObject::Response& response);

  bool extractSegmentsCallback(std_srvs::Empty::Request& request,
                               std_srvs::Empty::Response& response);

  void extractSegmentLayers(voxblox::Label label,
                            voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer,
                            voxblox::Layer<voxblox::LabelVoxel>* label_layer);

  bool lookupTransform(const std::string& from_frame,
                       const std::string& to_frame, const ros::Time& timestamp,
                       voxblox::Transformation* transform);

  ros::NodeHandle* node_handle_private_;

  tf::TransformListener tf_listener_;
  ros::Time last_segment_msg_timestamp_;

  // Shutdown logic: if no messages are received for X amount of time, shut down
  // node.
  bool received_first_message_;
  ros::Time last_update_received_;

  ros::Publisher* mesh_pub_;
  ros::Publisher* scene_pub_;
  ros::Publisher* object_pub_;
  ros::Publisher* gsm_update_pub_;
  ros::Timer update_mesh_timer_;

  size_t integrated_frames_count_;

  voxblox::LabelTsdfMap::Config map_config_;

  std::shared_ptr<voxblox::LabelTsdfMap> map_;
  std::shared_ptr<voxblox::LabelTsdfIntegrator> integrator_;

  voxblox::MeshLabelIntegrator::Config mesh_config_;

  std::shared_ptr<voxblox::MeshLayer> mesh_layer_;
  std::shared_ptr<voxblox::MeshLabelIntegrator> mesh_integrator_;

  std::vector<voxblox::Segment*> segments_to_integrate_;
  std::map<voxblox::Label, std::map<voxblox::Segment*, size_t>>
      segment_label_candidates;

  std::set<voxblox::Label> all_published_segments_;
  std::vector<voxblox::Label> segment_labels_to_publish_;

  std::map<voxblox::Label, std::set<voxblox::Label>> merges_to_publish_;
};
}  // namespace voxblox_gsm
}  // namespace voxblox

#endif  // VOXBLOX_GSM_INCLUDE_VOXBLOX_GSM_CONTROLLER_H_
