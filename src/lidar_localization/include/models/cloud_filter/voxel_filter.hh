#pragma once
/**
 *  点云滤波类
 */

#include <pcl/filters/voxel_grid.h>
#include "cloud_filter_interface.hh"

namespace lh {
class VoxelFilter : public CloudFilterInterface {
 public:
  VoxelFilter(const YAML::Node& node);
  VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z);

  bool Filter(const PointCloudPtr& input_cloud_ptr,
              PointCloudPtr& filtered_cloud_ptr) override;

 private:
  bool setFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z);

 private:
  pcl::VoxelGrid<PointXYZ> voxel_filter_;
};
}  // namespace lh