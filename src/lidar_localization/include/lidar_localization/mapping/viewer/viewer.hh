#pragma once

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <string>

#include "lidar_localization/models/cloud_filter/voxel_filter.hpp"
#include "lidar_localization/sensor_data/cloud_data.hpp"
#include "lidar_localization/sensor_data/key_frame.hpp"
#include "lidar_localization/sensor_data/pose_data.hpp"

namespace lidar_localization {
class Viewer {
 public:
  Viewer() = default;

 private:
  std::string data_path_ = "";
  std::string key_frames_path_ = "";
  std::map_path_ = "";
  int local_frame_num_ = 20;

  std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
  std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
  std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;

  Eigen::Matrix4f pose_to_optimized_ = Eigen::Matrix4f::Identity();
  PoseData optimized_odom_;
  CloudData optimized_cloud_;
  std::deque<KeyFrame> optimized_key_frames_;
  std::deque<KeyFrame> all_key_frames_;

  bool has_new_global_map_ = false;
  bool has_new_local_map_ = false;
};
}  // namespace lidar_localization