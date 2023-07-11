#pragma once

/**
 * 存放处理后的IMU姿态以及GNSS位置
 */

#include <Eigen/Dense>

namespace lidar_localization {
class PoseData {
 public:
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  double time = 0.0;

 public:
  Eigen::Quaternionf GetQuaternion();
};
}  // namespace lidar_localization