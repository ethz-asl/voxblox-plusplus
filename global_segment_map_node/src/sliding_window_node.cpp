#include "voxblox_gsm/controller.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

/*!
 * Serves as a wrapper for the standard gsm_node. It makes the gsm_node work
 * similar to a sliding window of window size t.
 * It basically restarts the gsm every t seconds.
 */
class SlidingWindow {
 public:
  /*!
   * @param node_handle needs to have aprivate parameter window_size_t
   */
  SlidingWindow(ros::NodeHandle& node_handle);
  void setupControllerServices();
  void setupController();
  /*!
   * Closes the old window and starts a new one. Also it notifies a downstream
   * node of this event, via a service call.
   * Called every window_size_t seconds
   */
  void newWindowCallback(const ros::TimerEvent&);
  /*!
   * Publishes the scene, all gsm_updates and the deletes the controller and
   * deregisters all services and publishers.
   * The node itself stays alive. A new controller is subsequently setup and
   * registered, including the new gsm.
   */
  void resetController();
  /*!
   * Deregisters all services and publishers.
   */
  void shutdownController();


 private:
  ros::NodeHandle node_handle_;

  std::unique_ptr<voxblox::voxblox_gsm::Controller> controller_;

  ros::Subscriber segment_point_cloud_sub_;

  ros::Publisher gsm_update_publisher_;
  ros::Publisher mesh_publisher_;
  ros::Publisher object_publisher_;
  ros::Publisher scene_publisher_;

  ros::ServiceServer extract_segments_srv_;
  ros::ServiceServer generate_mesh_srv_;
  ros::ServiceServer publish_scene_srv_;
  ros::ServiceServer validate_merged_object_srv_;

  ros::ServiceClient new_window_client_;

  ros::Timer window_timer_;
};

SlidingWindow::SlidingWindow(ros::NodeHandle& node_handle)
    : node_handle_(node_handle) {
  controller_ = std::unique_ptr<voxblox::voxblox_gsm::Controller>(
      new voxblox::voxblox_gsm::Controller(&node_handle));
  setupController();
  setupControllerServices();
  size_t window_size = node_handle.param("sliding_window_size_t", 30);

  const std::string service_name = "/loop_closure_node/new_session";
  new_window_client_ = node_handle_.serviceClient<std_srvs::Empty>(service_name);
  window_timer_ = node_handle.createTimer(
      ros::Duration(window_size), &SlidingWindow::newWindowCallback, this);
}

void SlidingWindow::setupControllerServices() {
  controller_->advertiseGenerateMeshService(&generate_mesh_srv_);
  controller_->advertisePublishSceneService(&publish_scene_srv_);
  controller_->advertiseExtractSegmentsService(&extract_segments_srv_);
  controller_->validateMergedObjectService(&validate_merged_object_srv_);
}

void SlidingWindow::setupController() {
  controller_->subscribeSegmentPointCloudTopic(&segment_point_cloud_sub_);
  controller_->advertiseSegmentMeshTopic(&mesh_publisher_);
  controller_->advertiseSceneMeshTopic(&scene_publisher_);
  controller_->advertiseSegmentGsmUpdateTopic(&object_publisher_);
  controller_->advertiseSceneGsmUpdateTopic(&gsm_update_publisher_);
}

void SlidingWindow::shutdownController() {
  segment_point_cloud_sub_.shutdown();
  mesh_publisher_.shutdown();
  scene_publisher_.shutdown();
  object_publisher_.shutdown();
  gsm_update_publisher_.shutdown();

  generate_mesh_srv_.shutdown();
  publish_scene_srv_.shutdown();
  extract_segments_srv_.shutdown();
  validate_merged_object_srv_.shutdown();
}

void SlidingWindow::resetController() {
  controller_->publishScene();
  controller_->publishObjects(true);
  shutdownController();

  controller_ = std::unique_ptr<voxblox::voxblox_gsm::Controller>(
      new voxblox::voxblox_gsm::Controller(&node_handle_));
  setupController();
  setupControllerServices();
}

void SlidingWindow::newWindowCallback(const ros::TimerEvent&) {
  LOG(INFO) << "Starting new window";
  resetController();
  std_srvs::Empty new_window;
  new_window_client_.call(new_window);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "gsm_sliding_window_node");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  LOG(INFO) << "Running GSM...";

  ros::NodeHandle node_handle;
  ros::NodeHandle node_handle_private("~");

  SlidingWindow window(node_handle_private);
  while (ros::ok()) {
    ros::spin();
  }
  LOG(INFO) << "Shutting down";
  return 0;
}
