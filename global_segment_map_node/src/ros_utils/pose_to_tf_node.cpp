#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <glog/logging.h>
#include <modelify/object_toolbox/common.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>

namespace modelify {
namespace loop_closure {
class PoseToTfNode {
 public:
  PoseToTfNode(ros::NodeHandle& node_handle);
  void newPoseCallback(const geometry_msgs::PoseStamped& pose_msg);
  void PoseStampedToTransformStamped(const geometry_msgs::PoseStamped& pose,
                                     tf::StampedTransform* tf);

  ros::NodeHandle node_handle_;
  ros::Subscriber markers_sub;
  tf::TransformBroadcaster tf_broadcaster;
  tf2_ros::StaticTransformBroadcaster tf_static;
  tf::TransformListener tf_listener_;

  std::string world_frame_ = "world";
  std::string map_frame_ = "map";
  std::string imu_estimated_frame_ = "imu_estimated";
  std::string depth_estimated_frame_ = "depth_estimated";
};

PoseToTfNode::PoseToTfNode(ros::NodeHandle& node_handle)
    : node_handle_(node_handle) {
  std::vector<double> translation;
  std::vector<double> rotation;
  node_handle_.getParam("/rovioli_marker_to_tf/T_I_D/quaternion", rotation);
  node_handle_.getParam("/rovioli_marker_to_tf/T_I_D/translation_m", translation);

  std::string topic = "/maplab_rovio/T_G_I";
  node_handle.param<std::string>("/pose_to_tf/pose_topic", topic, topic);
  markers_sub =
      node_handle_.subscribe(topic, 2000, &PoseToTfNode::newPoseCallback, this);

  tf::Quaternion rot(rotation[0], rotation[1], rotation[2], rotation[3]);
  tf::Vector3 trans(translation[0], translation[1], translation[2]);
  geometry_msgs::TransformStamped tf_imu_depth;
  tf_imu_depth.transform.rotation.x = rot.x();
  tf_imu_depth.transform.rotation.y = rot.y();
  tf_imu_depth.transform.rotation.z = rot.z();
  tf_imu_depth.transform.rotation.w = rot.w();
  tf_imu_depth.transform.translation.x = trans.x();
  tf_imu_depth.transform.translation.y = trans.y();
  tf_imu_depth.transform.translation.z = trans.z();
  tf_imu_depth.header.frame_id = imu_estimated_frame_;
  tf_imu_depth.child_frame_id = depth_estimated_frame_;
  tf_static.sendTransform(tf_imu_depth);

  geometry_msgs::TransformStamped tf_world_map;
  tf_world_map.transform.translation.x = 0;
  tf_world_map.transform.translation.y = 0;
  tf_world_map.transform.translation.z = 0;
  tf_world_map.transform.rotation.x = 0;
  tf_world_map.transform.rotation.y = 0;
  tf_world_map.transform.rotation.z = 0;
  tf_world_map.transform.rotation.w = 1;
  tf_world_map.header.frame_id = map_frame_;
  tf_world_map.child_frame_id = world_frame_;
  tf_static.sendTransform(tf_world_map);
}

void PoseToTfNode::newPoseCallback(const geometry_msgs::PoseStamped& pose_msg) {
  tf::StampedTransform T_W_I;
  PoseStampedToTransformStamped(pose_msg, &T_W_I);
  tf_broadcaster.sendTransform(T_W_I);
}

void PoseToTfNode::PoseStampedToTransformStamped(
    const geometry_msgs::PoseStamped& pose, tf::StampedTransform* tf) {
  tf->child_frame_id_ = imu_estimated_frame_;
  tf->frame_id_ = pose.header.frame_id;
  tf->stamp_ = pose.header.stamp;
  tf->setRotation(
      tf::Quaternion(pose.pose.orientation.x, pose.pose.orientation.y,
                     pose.pose.orientation.z, pose.pose.orientation.w));
  tf->setOrigin(tf::Vector3(pose.pose.position.x,
                            pose.pose.position.y,
                            pose.pose.position.z));
}

}
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "pose_to_tf");
  ros::NodeHandle node_handle;
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = 0;
  LOG(INFO) << "Started converter marker to tf.";
  modelify::loop_closure::PoseToTfNode node(node_handle);

  ros::spin();
}
