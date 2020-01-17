#ifndef VOXBLOX_GSM_CONVERSIONS_H_
#define VOXBLOX_GSM_CONVERSIONS_H_

#include <vector>

#include <geometry_msgs/Transform.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <voxblox/core/common.h>
#include <voxblox/io/sdf_ply.h>

namespace voxblox {
namespace voxblox_gsm {

inline void fillAlignedBoundingBoxMsg(Eigen::Vector3f bbox_translation,
                                      Eigen::Quaternionf bbox_quaternion,
                                      Eigen::Vector3f bbox_size,
                                      vpp_msgs::BoundingBox* bounding_box_msg) {
  CHECK_NOTNULL(bounding_box_msg);
  bounding_box_msg->pose.position.x = bbox_translation(0);
  bounding_box_msg->pose.position.y = bbox_translation(1);
  bounding_box_msg->pose.position.z = bbox_translation(2);
  bounding_box_msg->pose.orientation.x = bbox_quaternion.x();
  bounding_box_msg->pose.orientation.y = bbox_quaternion.y();
  bounding_box_msg->pose.orientation.z = bbox_quaternion.z();
  bounding_box_msg->pose.orientation.w = bbox_quaternion.w();
  bounding_box_msg->dimensions.x = bbox_size(0);
  bounding_box_msg->dimensions.y = bbox_size(1);
  bounding_box_msg->dimensions.z = bbox_size(2);
}

inline void fillBoundingBoxMarkerMsg(std::string world_frame, uint32_t id,
                                     Eigen::Vector3f bbox_translation,
                                     Eigen::Quaternionf bbox_quaternion,
                                     Eigen::Vector3f bbox_size,
                                     visualization_msgs::Marker* bbox_marker) {
  CHECK_NOTNULL(bbox_marker);
  bbox_marker->header.frame_id = world_frame;
  bbox_marker->header.stamp = ros::Time();
  bbox_marker->id = id;
  bbox_marker->type = visualization_msgs::Marker::CUBE;
  bbox_marker->action = visualization_msgs::Marker::ADD;
  bbox_marker->pose.position.x = bbox_translation(0);
  bbox_marker->pose.position.y = bbox_translation(1);
  bbox_marker->pose.position.z = bbox_translation(2);
  bbox_marker->pose.orientation.x = bbox_quaternion.x();
  bbox_marker->pose.orientation.y = bbox_quaternion.y();
  bbox_marker->pose.orientation.z = bbox_quaternion.z();
  bbox_marker->pose.orientation.w = bbox_quaternion.w();
  bbox_marker->scale.x = bbox_size(0);
  bbox_marker->scale.y = bbox_size(1);
  bbox_marker->scale.z = bbox_size(2);
  bbox_marker->color.a = 0.3;
  bbox_marker->color.r = 0.0;
  bbox_marker->color.g = 1.0;
  bbox_marker->color.b = 0.0;
  bbox_marker->lifetime = ros::Duration();
}

inline void fillBoundingBoxTfMsg(std::string world_frame,
                                 std::string child_frame,
                                 Eigen::Vector3f bbox_translation,
                                 Eigen::Quaternionf bbox_quaternion,
                                 geometry_msgs::TransformStamped* bbox_tf) {
  bbox_tf->header.stamp = ros::Time();
  bbox_tf->header.frame_id = world_frame;
  bbox_tf->child_frame_id = child_frame;
  bbox_tf->transform.translation.x = bbox_translation(0);
  bbox_tf->transform.translation.y = bbox_translation(1);
  bbox_tf->transform.translation.z = bbox_translation(2);
  bbox_tf->transform.rotation.x = bbox_quaternion.x();
  bbox_tf->transform.rotation.y = bbox_quaternion.y();
  bbox_tf->transform.rotation.z = bbox_quaternion.z();
  bbox_tf->transform.rotation.w = bbox_quaternion.w();
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

}  // namespace voxblox_gsm
}  // namespace voxblox
#endif  // VOXBLOX_GSM_CONVERSIONS_H_
