#pragma once
#include <Eigen/Dense>

namespace lidar_localization {
class KeyFrame {
 public:
  double time = 0.0;
  unsigned int index = 0.0;
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

 public:
  Eigen::Quaternionf GetQuaternion();
};
}  // namespace lidar_localization