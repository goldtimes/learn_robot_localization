#pragma once

#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include "glog/logging.h"

#include "lidar_localization/sensor_data/cloud_data.hpp"
#include "lidar_localization/sensor_data/velocity_data.hpp"

namespace lidar_localization {
class LidarUndistortion {
 public:
  void setMotionInfo(float scan_period, const VelocityData& velocity_data);
  bool undistortion(const CloudData::CLOUD_PTR& input_cloud_ptr,
                    CloudData::CLOUD_PTR& out_cloud_ptr);

 private:
  inline Eigen::Matrix3f updateMatrix(float real_time);

 private:
  float scan_period_;
  Eigen::Vector3f velocity_;
  Eigen::Vector3f angular_rate_;
};
}  // namespace lidar_localization