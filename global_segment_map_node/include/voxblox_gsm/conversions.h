#ifndef VOXBLOX_GSM_CONVERSIONS_H_
#define VOXBLOX_GSM_CONVERSIONS_H_

#include <vector>

#include <geometry_msgs/Transform.h>
#include <modelify_msgs/VoxelEvaluationDetails.h>
#include <pcl/point_types.h>
#include <voxblox/core/common.h>
#include <voxblox/io/sdf_ply.h>

namespace voxblox {
namespace voxblox_gsm {

inline void transformMsgs2Transformations(
    const std::vector<geometry_msgs::Transform>& transforms,
    std::vector<Transformation>* transformations) {
  CHECK_NOTNULL(transformations);
  for (const geometry_msgs::Transform& transform : transforms) {
    voxblox::Rotation quaternion(transform.rotation.w, transform.rotation.x,
                                 transform.rotation.y, transform.rotation.z);
    voxblox::Point translation(transform.translation.x, transform.translation.y,
                               transform.translation.z);
    transformations->emplace_back(quaternion, translation);
  }
}
inline void voxelEvaluationDetails2VoxelEvaluationDetailsMsg(
    const std::vector<voxblox::utils::VoxelEvaluationDetails>&
        voxel_evaluation_details_vector,
    std::vector<modelify_msgs::VoxelEvaluationDetails>*
        voxel_evaluation_details_msgs) {
  CHECK_NOTNULL(voxel_evaluation_details_msgs)->clear();

  for (const voxblox::utils::VoxelEvaluationDetails& voxel_evaluation_details :
       voxel_evaluation_details_vector) {
    // TODO(ff): Move this to voxblox_ros conversions.h.
    modelify_msgs::VoxelEvaluationDetails voxel_evaluation_details_msg;
    voxel_evaluation_details_msg.rmse = voxel_evaluation_details.rmse;
    voxel_evaluation_details_msg.max_error = voxel_evaluation_details.max_error;
    voxel_evaluation_details_msg.min_error = voxel_evaluation_details.min_error;
    voxel_evaluation_details_msg.num_evaluated_voxels =
        voxel_evaluation_details.num_evaluated_voxels;
    voxel_evaluation_details_msg.num_ignored_voxels =
        voxel_evaluation_details.num_ignored_voxels;
    voxel_evaluation_details_msg.num_overlapping_voxels =
        voxel_evaluation_details.num_overlapping_voxels;
    voxel_evaluation_details_msg.num_non_overlapping_voxels =
        voxel_evaluation_details.num_non_overlapping_voxels;
    voxel_evaluation_details_msgs->push_back(voxel_evaluation_details_msg);
  }
}

inline void convertVoxelGridToPointCloud(
    const voxblox::Layer<voxblox::TsdfVoxel>& tsdf_voxels,
    const MeshIntegratorConfig& mesh_config,
    pcl::PointCloud<pcl::PointSurfel>* surfel_cloud) {
  CHECK_NOTNULL(surfel_cloud);

  static constexpr bool kConnectedMesh = false;
  voxblox::Mesh mesh;
  io::convertLayerToMesh(tsdf_voxels, mesh_config, &mesh, kConnectedMesh);

  surfel_cloud->reserve(mesh.vertices.size());

  size_t vert_idx = 0u;
  for (const voxblox::Point& vert : mesh.vertices) {
    pcl::PointSurfel point;
    point.x = vert(0);
    point.y = vert(1);
    point.z = vert(2);

    if (mesh.hasColors()) {
      const voxblox::Color& color = mesh.colors[vert_idx];
      point.r = static_cast<int>(color.r);
      point.g = static_cast<int>(color.g);
      point.b = static_cast<int>(color.b);
    }

    if (mesh.hasNormals()) {
      const voxblox::Point& normal = mesh.normals[vert_idx];
      point.normal_x = normal(0);
      point.normal_y = normal(1);
      point.normal_z = normal(2);
    } else {
      LOG(FATAL) << "Mesh doesn't have normals.";
    }

    surfel_cloud->push_back(point);
    ++vert_idx;
  }

  surfel_cloud->is_dense = true;
  surfel_cloud->width = surfel_cloud->points.size();
  surfel_cloud->height = 1u;
}

bool convertTsdfLabelLayersToMesh(
    const Layer<TsdfVoxel>& tsdf_layer, const Layer<LabelVoxel>& label_layer,
    voxblox::Mesh* mesh, const bool connected_mesh = true,
    const FloatingPoint vertex_proximity_threshold = 1e-10) {
  CHECK_NOTNULL(mesh);

  MeshIntegratorConfig mesh_config;
  MeshLayer mesh_layer(tsdf_layer.block_size());
  std::set<SemanticLabel> all_semantic_labels;
  MeshLabelIntegrator mesh_integrator(mesh_config, tsdf_layer, label_layer,
                                      &mesh_layer, all_semantic_labels);

  // Generate mesh layer.
  constexpr bool only_mesh_updated_blocks = false;
  constexpr bool clear_updated_flag = false;
  mesh_integrator.generateMesh(only_mesh_updated_blocks, clear_updated_flag);

  // Extract mesh from mesh layer, either by simply concatenating all meshes
  // (there is one per block) or by connecting them.
  if (connected_mesh) {
    mesh_layer.getConnectedMesh(mesh, vertex_proximity_threshold);
  } else {
    mesh_layer.getMesh(mesh);
  }
  return mesh->size() > 0u;
}

}  // namespace voxblox_gsm
}  // namespace voxblox
#endif  // VOXBLOX_GSM_CONVERSIONS_H_
