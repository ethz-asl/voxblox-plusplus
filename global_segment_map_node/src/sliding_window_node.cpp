//
// Created by sebastian on 4/8/18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>

#include "std_srvs/Empty.h"
#include "voxblox_gsm/controller.h"

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

  std::unique_ptr<voxblox::voxblox_gsm::Controller> controller;

 private:
  ros::NodeHandle node_handle_;

  ros::Subscriber segment_point_cloud_sub;
  ros::ServiceServer validate_merged_object_srv;
  ros::Publisher mesh_publisher;
  ros::Publisher scene_publisher;
  ros::Publisher object_publisher;
  ros::Publisher gsm_update_publisher;

  ros::ServiceServer generate_mesh_srv;
  ros::ServiceServer publish_scene_srv;
  ros::ServiceServer extract_segments_srv;

  ros::Timer window_timer;
  ros::ServiceClient new_window_client;
};

SlidingWindow::SlidingWindow(ros::NodeHandle& node_handle)
    : node_handle_(node_handle) {
  controller = std::unique_ptr<voxblox::voxblox_gsm::Controller>(
      new voxblox::voxblox_gsm::Controller(&node_handle));
  setupController();
  setupControllerServices();
  int window_size = node_handle.param("sliding_window_size_t", 5);
  std::cout << window_size << " window size" << std::endl;

  const std::string service_name = "/loop_closure_node/new_session";
  new_window_client = node_handle_.serviceClient<std_srvs::Empty>(service_name);
  window_timer = node_handle.createTimer(
      ros::Duration(window_size), &SlidingWindow::newWindowCallback, this);
}

void SlidingWindow::setupControllerServices() {
  controller->advertiseGenerateMeshService(&generate_mesh_srv);
  controller->advertisePublishSceneService(&publish_scene_srv);
  controller->advertiseExtractSegmentsService(&extract_segments_srv);
  controller->validateMergedObjectService(&validate_merged_object_srv);
}

void SlidingWindow::setupController() {
  controller->subscribeSegmentPointCloudTopic(&segment_point_cloud_sub);
  controller->advertiseSegmentMeshTopic(&mesh_publisher);
  controller->advertiseSceneMeshTopic(&scene_publisher);
  controller->advertiseSegmentGsmUpdateTopic(&object_publisher);
  controller->advertiseSceneGsmUpdateTopic(&gsm_update_publisher);
}

void SlidingWindow::shutdownController() {
  segment_point_cloud_sub.shutdown();
  mesh_publisher.shutdown();
  scene_publisher.shutdown();
  object_publisher.shutdown();
  gsm_update_publisher.shutdown();

  generate_mesh_srv.shutdown();
  publish_scene_srv.shutdown();
  extract_segments_srv.shutdown();
  validate_merged_object_srv.shutdown();
}

void SlidingWindow::resetController() {
  // publish scene
  controller->publishScene();
  controller->publishObjects(true);
  shutdownController();

  controller = std::unique_ptr<voxblox::voxblox_gsm::Controller>(
      new voxblox::voxblox_gsm::Controller(&node_handle_));
  setupController();
  setupControllerServices();
}

void SlidingWindow::newWindowCallback(const ros::TimerEvent&) {
  LOG(INFO) << "Starting new window" << endl;
  resetController();
  std_srvs::Empty new_window;
  new_window_client.call(new_window);
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
  constexpr double kNoUpdateTimeout = 5.0;
  while (ros::ok()) {
    ros::spin();
  }
  LOG(INFO) << "Shutting down";
  return 0;
}
