/*
 * @Description: voxel filter 模块
 * @Author: Ren Qian
 * @Date: 2020-02-09 19:37:49
 */
#ifndef LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_VOXEL_FILTER_HPP_
#define LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_VOXEL_FILTER_HPP_

#include <pcl/filters/voxel_grid.h>
#include "models/cloud_filter/cloud_filter_interface.hpp"

namespace lh {
class VoxelFilter : public CloudFilterInterface {
 public:
  VoxelFilter(const YAML::Node& node);
  VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z);

  bool Filter(const PointCloudPtr& input_cloud_ptr,
              PointCloudPtr& filtered_cloud_ptr) override;

 private:
  bool SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z);

 private:
  pcl::VoxelGrid<PointXYZ> voxel_filter_;
};
}  // namespace lh
#endif