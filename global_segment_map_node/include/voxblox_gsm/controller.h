// Copyright 2018 Margarita Grinvald, ASL, ETH Zurich, Switzerland

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

typedef std::pair<Layer<TsdfVoxel>, Layer<LabelVoxel>> LayerPair;

class Controller {
 public:
  Controller(ros::NodeHandle* node_handle);

  ~Controller();

  void subscribeSegmentPointCloudTopic(
      ros::Subscriber* segment_point_cloud_sub);

  void advertiseSegmentGsmUpdateTopic(ros::Publisher* segment_gsm_update_pub);

  void advertiseSceneGsmUpdateTopic(ros::Publisher* scene_gsm_update_pub);

  void advertiseSegmentMeshTopic(ros::Publisher* segment_mesh_pub);

  void advertiseSceneMeshTopic(ros::Publisher* scene_mesh_pub);

  void advertisePublishSceneService(ros::ServiceServer* publish_scene_srv);

  void validateMergedObjectService(
      ros::ServiceServer* validate_merged_object_srv);

  void advertiseGenerateMeshService(ros::ServiceServer* generate_mesh_srv);

  void advertiseExtractSegmentsService(
      ros::ServiceServer* extract_segments_srv);

  bool publishObjects(const bool publish_all = false);

  void publishScene();

  bool noNewUpdatesReceived() const;

  bool publish_gsm_updates_;

  bool publish_scene_mesh_;
  bool publish_segment_mesh_;

  double no_update_timeout_;

 protected:
  virtual void segmentPointCloudCallback(
      const sensor_msgs::PointCloud2::Ptr& segment_point_cloud_msg);

  virtual bool publishSceneCallback(std_srvs::Empty::Request& request,
                                    std_srvs::Empty::Response& response);

  bool validateMergedObjectCallback(
      modelify_msgs::ValidateMergedObject::Request& request,
      modelify_msgs::ValidateMergedObject::Response& response);

  bool generateMeshCallback(std_srvs::Empty::Request& request,
                            std_srvs::Empty::Response& response);

  bool extractSegmentsCallback(std_srvs::Empty::Request& request,
                               std_srvs::Empty::Response& response);

  /**
   * Extracts separate tsdf and label layers from the gsm, for every given
   * label.
   * @param labels of segments to extract
   * @param label_layers_map output map
   * @param labels_list_is_complete true if the gsm does not contain other
   * labels. false if \labels is only a subset of all labels contained by the
   * gsm.
   */
  virtual void extractSegmentLayers(
      const std::vector<Label> &labels,
      std::unordered_map<Label, LayerPair> *label_layers_map,
      bool labels_list_is_complete = false);

  bool lookupTransform(const std::string& from_frame,
                       const std::string& to_frame, const ros::Time& timestamp,
                       Transformation* transform);

  void generateMesh(bool clear_mesh);

  void updateMeshEvent(const ros::TimerEvent& e);

  virtual void publishGsmUpdate(const ros::Publisher& publisher,
                                modelify_msgs::GsmUpdate& gsm_update);

  virtual void getLabelsToPublish(std::vector<Label>* labels, bool get_all);

  ros::NodeHandle* node_handle_private_;

  tf::TransformListener tf_listener_;
  ros::Time last_segment_msg_timestamp_;
  size_t integrated_frames_count_;

  // Shutdown logic: if no messages are received for X amount of time,
  // shut down node.
  bool received_first_message_;
  ros::Time last_update_received_;

  ros::Publisher* scene_gsm_update_pub_;
  ros::Publisher* segment_gsm_update_pub_;

  ros::Timer update_mesh_timer_;
  ros::Publisher* scene_mesh_pub_;
  ros::Publisher* segment_mesh_pub_;
  std::string mesh_filename_;

  std::string world_frame_;
  std::string camera_frame_;

  LabelTsdfMap::Config map_config_;

  std::shared_ptr<LabelTsdfMap> map_;
  std::shared_ptr<LabelTsdfIntegrator> integrator_;

  MeshIntegratorConfig mesh_config_;

  std::shared_ptr<MeshLayer> mesh_layer_;
  std::shared_ptr<MeshLabelIntegrator> mesh_integrator_;

  std::vector<Segment*> segments_to_integrate_;
  std::map<Label, std::map<Segment*, size_t>> segment_label_candidates;
  std::map<Segment*, std::vector<Label>> segment_merge_candidates_;

  std::set<Label> all_published_segments_;
  std::vector<Label> segment_labels_to_publish_;

  std::map<Label, std::set<Label>> merges_to_publish_;
};
}  // namespace voxblox_gsm
}  // namespace voxblox

#endif  // VOXBLOX_GSM_INCLUDE_VOXBLOX_GSM_CONTROLLER_H_
