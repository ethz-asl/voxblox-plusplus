#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <glog/logging.h>
#include <modelify/object_toolbox/common.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
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
  tf::TransformListener tf_listener_;

  std::string world_frame_ = "world";
  std::string map_frame_ = "map";
  std::string imu_frame_ = "imu";
  std::string depth_estimated_frame_ = "depth_estimated";
  tf::Transform T_I_D;
};

PoseToTfNode::PoseToTfNode(ros::NodeHandle& node_handle)
    : node_handle_(node_handle) {
  LOG(WARNING) << "Starting node";
  std::vector<double> translation;
  std::vector<double> rotation;
  node_handle_.getParam("/rovioli_marker_to_tf/T_I_D/quaternion", rotation);
  node_handle_.getParam("/rovioli_marker_to_tf/T_I_D/translation_m", translation);


  LOG(WARNING) << "Got params";
  tf::Quaternion rot(rotation[0], rotation[1], rotation[2], rotation[3]);
  tf::Vector3 trans(translation[0], translation[1], translation[2]);
  T_I_D.setRotation(rot);
  T_I_D.setOrigin(trans);

  LOG(WARNING) << "Tf set";
  std::string topic = "/maplab_rovio/T_G_I";
  node_handle.param<std::string>("/pose_to_tf/pose_topic", topic, topic);
  markers_sub =
      node_handle_.subscribe(topic, 2000, &PoseToTfNode::newPoseCallback, this);
  LOG(WARNING) << "finish setup";
}

void PoseToTfNode::newPoseCallback(const geometry_msgs::PoseStamped& pose_msg) {
  tf::StampedTransform T_W_I;
  PoseStampedToTransformStamped(pose_msg, &T_W_I);
  tf::Transform T_W_D;
  T_W_D = T_W_I * T_I_D;

  tf::StampedTransform T_W_D_msg;
  T_W_D_msg.setData(T_W_D);
  T_W_D_msg.frame_id_ = world_frame_;
  T_W_D_msg.child_frame_id_ = depth_estimated_frame_;
  T_W_D_msg.stamp_ = T_W_I.stamp_;

  tf_broadcaster.sendTransform(T_W_D_msg);

  tf::StampedTransform tf_world_map;
  tf_world_map.setIdentity();
  tf_world_map.frame_id_ = world_frame_;
  tf_world_map.child_frame_id_ = map_frame_;
  tf_world_map.stamp_ = pose_msg.header.stamp;
  tf_broadcaster.sendTransform(tf_world_map);
}

void PoseToTfNode::PoseStampedToTransformStamped(
    const geometry_msgs::PoseStamped& pose, tf::StampedTransform* tf) {
  tf->child_frame_id_ = depth_estimated_frame_;
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