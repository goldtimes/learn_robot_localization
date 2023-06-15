#include "models/cloud_filter/voxel_filter.hh"
#include "glog/logging.h"

namespace lh {
VoxelFilter::VoxelFilter(const YAML::Node& node) {
  float leaf_size_x = node["leaf_size"][0].as<float>();
  float leaf_size_y = node["leaf_size"][1].as<float>();
  float leaf_size_z = node["leaf_size"][2].as<float>();

  setFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
}

VoxelFilter::VoxelFilter(float leaf_size_x, float leaf_size_y,
                         float leaf_size_z) {
  setFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
}

bool VoxelFilter::setFilterParam(float leaf_size_x, float leaf_size_y,
                                 float leaf_size_z) {
  voxel_filter_.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
  LOG(INFO) << "Voxel Filter 的参数为:" << std::endl
            << leaf_size_x << ", " << leaf_size_y << ", " << leaf_size_z
            << std::endl;

  return true;
}

bool VoxelFilter::Filter(const PointCloudPtr& input_cloud_ptr,
                         PointCloudPtr& filtered_cloud_ptr) {
  voxel_filter_.setInputCloud(input_cloud_ptr);
  voxel_filter_.filter(*filtered_cloud_ptr);
  return true;
}

}  // namespace lh