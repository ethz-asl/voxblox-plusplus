// Copyright (c) 2019, ASL, ETH Zurich, Switzerland
// Licensed under the BSD 3-Clause License (see LICENSE for details)

#ifndef VOXBLOX_GSM_IODB_CONTROLLER_H_
#define VOXBLOX_GSM_IODB_CONTROLLER_H_

#include <global_feature_map/feature_block.h>
#include <global_feature_map/feature_integrator.h>
#include <global_feature_map/feature_layer.h>
#include <global_feature_map/feature_types.h>
#include <global_feature_map/feature_utils.h>
#include <global_segment_map_node/controller.h>
#include <modelify_msgs/Features.h>
#include <modelify_msgs/GsmUpdate.h>
#include <modelify_msgs/ValidateMergedObject.h>

namespace voxblox {
namespace voxblox_gsm {

typedef std::tuple<Layer<TsdfVoxel>, Layer<LabelVoxel>, FeatureLayer<Feature3D>>
    LayerTuple;

enum LayerAccessor {
  kTsdfLayer = 0,
  kLabelLayer = 1,
  kFeatureLayer = 2,
  kCount
};

class IodbController : public Controller {
 public:
  IodbController(ros::NodeHandle* node_handle);

  void subscribeFeatureTopic(ros::Subscriber* feature_sub);

  void advertiseSegmentMeshTopic();

  void advertiseFeatureBlockTopic();

  void advertiseSegmentGsmUpdateTopic();

  void advertiseSceneGsmUpdateTopic();

  void advertisePublishSceneService(ros::ServiceServer* publish_scene_srv);

  void validateMergedObjectService(
      ros::ServiceServer* validate_merged_object_srv);

  void extractSegmentLayers(
      const std::vector<Label>& labels,
      std::unordered_map<Label, LayerTuple>* label_layers_map,
      bool labels_list_is_complete = false);

  bool publishObjects(const bool publish_all = false);

  void publishScene();

  bool noNewUpdatesReceived() const;

  bool publish_gsm_updates_;
  bool publish_segment_mesh_;
  bool publish_feature_blocks_marker_;

  double no_update_timeout_;

 protected:
  virtual void segmentPointCloudCallback(
      const sensor_msgs::PointCloud2::Ptr& segment_point_cloud_msg);

  void featureCallback(const modelify_msgs::Features& features_msg);

  virtual bool publishSceneCallback(std_srvs::SetBool::Request& request,
                                    std_srvs::SetBool::Response& response);

  bool validateMergedObjectCallback(
      modelify_msgs::ValidateMergedObject::Request& request,
      modelify_msgs::ValidateMergedObject::Response& response);

  virtual void getLabelsToPublish(const bool get_all,
                                  std::vector<Label>* labels);

  virtual void publishGsmUpdate(const ros::Publisher& publisher,
                                modelify_msgs::GsmUpdate* gsm_update);

  ros::Publisher* scene_gsm_update_pub_;
  ros::Publisher* segment_gsm_update_pub_;
  std::set<Label> all_published_segments_;

  ros::Publisher* segment_mesh_pub_;

  int min_number_of_allocated_blocks_to_publish_;

  bool received_first_feature_;
  std::shared_ptr<FeatureLayer<Feature3D>> feature_layer_;
  std::shared_ptr<FeatureIntegrator> feature_integrator_;

  ros::Publisher* feature_block_pub_;

  // Shutdown logic: if no messages are received for X amount of time,
  // shut down node.
  ros::Time last_update_received_;
};

}  // namespace voxblox_gsm
}  // namespace voxblox

#endif  // VOXBLOX_GSM_IODB_CONTROLLER_H_
