#include "global_segment_map/utils/visualizer.h"

namespace voxblox {

Visualizer::Visualizer(
    const std::vector<std::shared_ptr<MeshLayer>>& mesh_layers,
    bool* mesh_layer_updated, std::mutex* mesh_layer_mutex_ptr,
    std::vector<double> camera_position, std::vector<double> clip_distances,
    bool save_visualizer_frames)
    : mesh_layers_(mesh_layers),
      mesh_layer_updated_(CHECK_NOTNULL(mesh_layer_updated)),
      mesh_layer_mutex_ptr_(CHECK_NOTNULL(mesh_layer_mutex_ptr)),
      frame_count_(0u),
      camera_position_(camera_position),
      clip_distances_(clip_distances),
      save_visualizer_frames_(save_visualizer_frames) {}

// TODO(grinvalm): make it more efficient by only updating the
// necessary polygons and not all of them each time.
void Visualizer::visualizeMesh() {
  uint8_t n_visualizers = mesh_layers_.size();

  std::vector<std::shared_ptr<pcl::visualization::PCLVisualizer>>
      pcl_visualizers;
  std::vector<voxblox::Mesh> meshes;
  std::vector<pcl::PointCloud<pcl::PointXYZRGBA>> pointclouds;
  std::vector<pcl::PolygonMesh> polygon_meshes;

  pcl_visualizers.reserve(n_visualizers);
  meshes.resize(n_visualizers);
  pointclouds.resize(n_visualizers);
  polygon_meshes.resize(n_visualizers);

  bool refresh = false;

  for (int index = 0; index < n_visualizers; ++index) {
    // PCLVisualizer class can NOT be used across multiple threads, thus need to
    // create instances of it in the same thread they will be used in.
    std::shared_ptr<pcl::visualization::PCLVisualizer> visualizer =
        std::make_shared<pcl::visualization::PCLVisualizer>();
    std::string name = "Map " + std::to_string(index + 1);
    visualizer->setWindowName(name.c_str());
    visualizer->setBackgroundColor(255, 255, 255);
    visualizer->initCameraParameters();

    if (camera_position_.size()) {
      visualizer->setCameraPosition(
          camera_position_[0], camera_position_[1], camera_position_[2],
          camera_position_[3], camera_position_[4], camera_position_[5],
          camera_position_[6], camera_position_[7], camera_position_[8]);
    }
    if (clip_distances_.size()) {
      visualizer->setCameraClipDistances(clip_distances_[0],
                                         clip_distances_[1]);
    }

    pcl_visualizers.push_back(visualizer);
  }

  while (true) {
    for (int index = 0; index < n_visualizers; ++index) {
      constexpr int kUpdateIntervalMs = 1000;
      pcl_visualizers[index]->spinOnce(kUpdateIntervalMs);
    }
    meshes.clear();
    meshes.resize(n_visualizers);

    if (mesh_layer_mutex_ptr_->try_lock()) {
      if (*mesh_layer_updated_) {
        for (int index = 0; index < n_visualizers; index++) {
          mesh_layers_[index]->getMesh(&meshes[index]);
        }
        refresh = true;
        *mesh_layer_updated_ = false;
      }
      mesh_layer_mutex_ptr_->unlock();
    }

    if (refresh) {
      for (int index = 0; index < n_visualizers; index++) {
        pointclouds[index].points.clear();
      }

      pcl::PCLPointCloud2 pcl_pc;
      std::vector<pcl::Vertices> polygons;

      for (int index = 0; index < n_visualizers; index++) {
        size_t vert_idx = 0;
        for (const Point& vert : meshes[index].vertices) {
          pcl::PointXYZRGBA point;
          point.x = vert(0);
          point.y = vert(1);
          point.z = vert(2);

          const Color& color = meshes[index].colors[vert_idx];
          point.r = color.r;
          point.g = color.g;
          point.b = color.b;
          point.a = color.a;
          pointclouds[index].points.push_back(point);

          vert_idx++;
        }

        for (size_t i = 0u; i < meshes[index].indices.size(); i += 3u) {
          pcl::Vertices face;
          for (int j = 0; j < 3; j++) {
            face.vertices.push_back(meshes[index].indices.at(i + j));
          }
          polygons.push_back(face);
        }

        pcl::toPCLPointCloud2(pointclouds[index], pcl_pc);
        polygon_meshes[index].cloud = pcl_pc;
        polygon_meshes[index].polygons = polygons;
      }

      for (int index = 0; index < n_visualizers; index++) {
        pcl_visualizers[index]->removePolygonMesh("meshes");
        if (!pcl_visualizers[index]->updatePolygonMesh(polygon_meshes[index],
                                                       "meshes")) {
          pcl_visualizers[index]->addPolygonMesh(polygon_meshes[index],
                                                 "meshes", 0);
        }

        if (save_visualizer_frames_) {
          pcl_visualizers[index]->saveScreenshot(
              "vpp_map_" + std::to_string(index) + "/frame_" +
              std::to_string(frame_count_) + ".png");
        }
      }
      frame_count_++;

      refresh = false;
    }
  }
}
}  // namespace voxblox
