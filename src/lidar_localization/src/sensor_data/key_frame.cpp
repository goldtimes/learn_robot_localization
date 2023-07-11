#include "lidar_localization/sensor_data/key_frame.hpp"

namespace lidar_localization {
Eigen::Quaternionf KeyFrame::GetQuaternion() {
  Eigen::Quaternionf q;
  //    直接拿旋转矩阵赋值给quaternion
  q = pose.block<3, 3>(0, 0);
  return q;
}
}  // namespace lidar_localization