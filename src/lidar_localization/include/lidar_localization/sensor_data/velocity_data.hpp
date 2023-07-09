/*
 * @Description: velocity 数据
 * @Author: Ren Qian
 * @Date: 2019-07-17 18:27:40
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_VELOCITY_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_VELOCITY_DATA_HPP_

#include <Eigen/Dense>
#include <deque>

namespace lidar_localization {
class VelocityData {
 public:
  struct LinearVelocity {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };

  struct AngularVelocity {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };

  double time = 0.0;
  LinearVelocity linear_velocity;
  AngularVelocity angular_velocity;

 public:
  static bool SyncData(std::deque<VelocityData>& UnsyncedData,
                       std::deque<VelocityData>& SyncedData, double sync_time);
  void TransformCoordinate(const Eigen::Matrix4f& lidar_to_imu);
};
}  // namespace lidar_localization
#endif