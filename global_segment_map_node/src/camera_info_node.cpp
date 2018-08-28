//
// Created by sebastian on 8/28/18.
//
#include <glog/logging.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

namespace camera_info {
  class CameraInfoNode {
  public:
    CameraInfoNode(ros::NodeHandle node_handle);
    void run(const sensor_msgs::Image& img);

  private:
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::NodeHandle node_handle_;
    sensor_msgs::CameraInfo msg;
  };

  CameraInfoNode::CameraInfoNode(ros::NodeHandle node_handle) : node_handle_(node_handle) {
    msg.header.frame_id = "camera_depth_optical_frame";
    msg.height = 480;
    msg.width = 640;
    msg.distortion_model = "plumb_bob";
    msg.D = {-0.26574138338630976, 0.06283680145030723, -0.002599238182830417,
             0.0002455757997017071};
    msg.K = {575.7716079915485, 0.0, 317.7512234343908, 0.0, 576.189809529098, 243.32258175147237, 0.0, 0.0, 1.0};
    msg.P = {575.7716079915485, 0.0, 317.7512234343908, 0.0, 0.0, 576.189809529098, 243.32258175147237, 0.0, 0.0, 0.0, 1.0, 0.0};
    pub_ = node_handle_.advertise<sensor_msgs::CameraInfo>(
        "/camera/depth/camera_info", 1, false);
    sub_ = node_handle.subscribe("/camera/depth_registered/image_raw", 1, &CameraInfoNode::run, this);
  }

  void CameraInfoNode::run(const sensor_msgs::Image& img) {
    msg.header.stamp = img.header.stamp;
    pub_.publish(msg);
  }
} // namespace camera_info

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "camera_info");
  ros::NodeHandle node_handle;
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = 0;
  LOG(INFO) << "Started converter marker to tf.";
  camera_info::CameraInfoNode node(node_handle);
  ros::spin();
}
