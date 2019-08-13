#ifndef GLOBAL_SEGMENT_MAP_UTILS_ICP_UTILS_H_
#define GLOBAL_SEGMENT_MAP_UTILS_ICP_UTILS_H_

#include <voxblox/alignment/icp.h>

namespace voxblox {

ICP::Config getICPConfigFromGflags();

bool refinePointCloudLayerTransform(const Layer<TsdfVoxel>& tsdf_layer,
                                    const Pointcloud& point_cloud,
                                    const Transformation& T_init, ICP* icp_p,
                                    Transformation* T_icp_p,
                                    Transformation* T_offset_p);

}  // namespace voxblox

#endif  // GLOBAL_SEGMENT_MAP_UTILS_ICP_UTILS_H_
