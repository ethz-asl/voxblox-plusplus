#ifndef GLOBAL_SEGMENT_MAP_UTILS_VISUALIZER_H_
#define GLOBAL_SEGMENT_MAP_UTILS_VISUALIZER_H_

#include <mutex>

#include <pcl/visualization/pcl_visualizer.h>
#include <voxblox/mesh/mesh_layer.h>

namespace voxblox {

class Visualizer {
 public:
  Visualizer(const std::vector<std::shared_ptr<MeshLayer>>& mesh_layers,
             bool* mesh_layer_updated, std::mutex* mesh_layer_mutex_ptr);

  void visualizeMesh();

  std::vector<std::shared_ptr<MeshLayer>> mesh_layers_;

  std::mutex* mesh_layer_mutex_ptr_;
  bool* mesh_layer_updated_;

  size_t frame_count_;
};
}  // namespace voxblox

#endif  // GLOBAL_SEGMENT_MAP_UTILS_VISUALIZER_H_
