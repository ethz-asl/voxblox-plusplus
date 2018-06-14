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

  void getAndPublishT_I_D();
  Eigen::Matrix4f getMatrix(std::vector<double> quaternions, std::vector<double> translation);
  void publishTransform(const Eigen::Matrix4f matrix, std::string frame_id, std::string child_frame_id);

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

  std::string topic = "/maplab_rovio/T_G_I";
  node_handle.param<std::string>("/pose_to_tf/pose_topic", topic, topic);
  markers_sub =
      node_handle_.subscribe(topic, 200000000, &PoseToTfNode::newPoseCallback, this);

  getAndPublishT_I_D();

  // T map to world
  Eigen::Matrix4f T_World_Map;
  T_World_Map.setIdentity();
  publishTransform(T_World_Map, world_frame_, map_frame_);
}

void PoseToTfNode::newPoseCallback(const geometry_msgs::PoseStamped& pose_msg) {
  tf::StampedTransform T_W_I;
  PoseStampedToTransformStamped(pose_msg, &T_W_I);
  tf_broadcaster.sendTransform(T_W_I);
}

void PoseToTfNode::PoseStampedToTransformStamped(
    const geometry_msgs::PoseStamped& pose, tf::StampedTransform* tf) {
  tf->frame_id_ = pose.header.frame_id;
  tf->child_frame_id_ = imu_estimated_frame_;
  tf->stamp_ = pose.header.stamp;
  tf->setRotation(
      tf::Quaternion(pose.pose.orientation.x, pose.pose.orientation.y,
                     pose.pose.orientation.z, pose.pose.orientation.w));
  tf->setOrigin(tf::Vector3(pose.pose.position.x,
                            pose.pose.position.y,
                            pose.pose.position.z));
}

Eigen::Matrix4f PoseToTfNode::getMatrix(std::vector<double> quaternions,
                                        std::vector<double> translation) {
  Eigen::Quaternionf quaternion(quaternions[3], quaternions[0], quaternions[1], quaternions[2]);
  Eigen::Vector3f base;
  Eigen::Matrix4f tf;
  tf.setIdentity();
  tf.block<3,3>(0,0) = quaternion.toRotationMatrix();
  tf.block<3,1>(0,3) << translation[0] , translation[1], translation[2];
  return tf;
}


void PoseToTfNode::getAndPublishT_I_D() {
  std::vector<double> translation;
  std::vector<double> rotation;

  // T Imu tango to Depth
  node_handle_.getParam("/rovioli_marker_to_tf/T_I_D/quaternion", rotation);
  node_handle_.getParam("/rovioli_marker_to_tf/T_I_D/translation_m", translation);
  Eigen::Matrix4f T_T_D = getMatrix(rotation, translation);

  // T imu tango to fisheye
  node_handle_.getParam("/rovioli_marker_to_tf/T_T_F/quaternion", rotation);
  node_handle_.getParam("/rovioli_marker_to_tf/T_T_F/translation_m", translation);
  Eigen::Matrix4f T_T_F = getMatrix(rotation, translation);

  // T imu rovioli to fisheye
  std::vector<double> matrix_as_vector;
  node_handle_.getParam("/rovioli_marker_to_tf/T_R_F/matrix", matrix_as_vector);

  Eigen::Matrix4f T_R_F;
  for (size_t i=0; i<4; ++i) {
    for (size_t j=0; j<4; ++j) {
      T_R_F(i,j) = matrix_as_vector[4*i + j];
    }
  }

  LOG(INFO) << "TID" << T_T_D << "TTF" << T_T_F << "TRF" << T_R_F;
  Eigen::Matrix4f T_R_D;
  T_R_D = T_R_F * T_T_F.inverse() * T_T_D;

  publishTransform(T_R_D, imu_estimated_frame_, depth_estimated_frame_);
}

void PoseToTfNode::publishTransform(const Eigen::Matrix4f matrix, std::string frame_id, std::string child_frame_id) {
  LOG(INFO) << "Publishing tf " << frame_id << " to " << child_frame_id << std::endl << matrix;
  geometry_msgs::TransformStamped tf;
  Eigen::Quaternionf quat(matrix.block<3,3>(0,0));
  tf.transform.rotation.x = quat.x();
  tf.transform.rotation.y = quat.y();
  tf.transform.rotation.z = quat.z();
  tf.transform.rotation.w = quat.w();
  tf.transform.translation.x = matrix(0,3);
  tf.transform.translation.y = matrix(1,3);
  tf.transform.translation.z = matrix(2,3);
  tf.header.frame_id = frame_id;
  tf.child_frame_id = child_frame_id;
  tf_static.sendTransform(tf);

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
