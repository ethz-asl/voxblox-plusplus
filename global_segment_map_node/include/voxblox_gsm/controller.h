// Copyright 2017 Margarita Grinvald, ASL, ETH Zurich, Switzerland

#ifndef VOXBLOX_GSM_INCLUDE_VOXBLOX_GSM_CONTROLLER_H_
#define VOXBLOX_GSM_INCLUDE_VOXBLOX_GSM_CONTROLLER_H_

#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <global_segment_map/label_tsdf_map.h>
#include <global_segment_map/label_tsdf_integrator.h>
#include <voxblox/io/mesh_ply.h>
#include <global_segment_map/label_tsdf_mesh_integrator.h>

namespace voxblox_gsm {

class Controller {
 public:
  Controller(ros::NodeHandle* node_handle);

  ~Controller();

  void subscribeSegmentPointCloudTopic(
      ros::Subscriber* segment_point_cloud_sub);

  void advertiseMeshTopic(ros::Publisher* mesh_pub);

  void advertiseGenerateMeshService(ros::ServiceServer* generate_mesh_srv);

  void advertiseExtractSegmentsService(
      ros::ServiceServer* extract_segments_srv);

  void updateMeshEvent(const ros::TimerEvent& e);

 private:
  void segmentPointCloudCallback(
      const sensor_msgs::PointCloud2::Ptr& segment_point_cloud_msg);

  bool generateMeshCallback(std_srvs::Empty::Request& request,
                            std_srvs::Empty::Response& response);

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

  ros::Publisher* mesh_pub_;
  ros::Timer update_mesh_timer_;

  size_t callback_count_;

  voxblox::LabelTsdfMap::Config map_config_;

  std::shared_ptr<voxblox::LabelTsdfMap> map_;
  std::shared_ptr<voxblox::LabelTsdfIntegrator> integrator_;

  voxblox::MeshLabelIntegrator::Config mesh_config_;

  std::shared_ptr<voxblox::MeshLayer> mesh_layer_;
  std::shared_ptr<voxblox::MeshLabelIntegrator> mesh_integrator_;

  std::vector<voxblox::Segment*> segments_to_integrate_;
  std::map<voxblox::Label, std::map<voxblox::Segment*, size_t> >
      segment_label_candidates;
};
}  // namespace voxblox_gsm

#endif  // VOXBLOX_GSM_INCLUDE_VOXBLOX_GSM_CONTROLLER_H_
