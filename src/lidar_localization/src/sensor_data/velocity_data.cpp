/*
 * @Description: velocity data
 * @Author: Ren Qian
 * @Date: 2020-02-23 22:20:41
 */
#include "lidar_localization/sensor_data/velocity_data.hpp"

#include "glog/logging.h"

namespace lidar_localization {
bool VelocityData::SyncData(std::deque<VelocityData>& UnsyncedData,
                            std::deque<VelocityData>& SyncedData,
                            double sync_time) {
  // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
  // 即找到与同步时间相邻的左右两个数据
  // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
  while (UnsyncedData.size() >= 2) {
    if (UnsyncedData.front().time > sync_time) return false;
    if (UnsyncedData.at(1).time < sync_time) {
      UnsyncedData.pop_front();
      continue;
    }
    if (sync_time - UnsyncedData.front().time > 0.2) {
      UnsyncedData.pop_front();
      break;
    }
    if (UnsyncedData.at(1).time - sync_time > 0.2) {
      UnsyncedData.pop_front();
      break;
    }
    break;
  }
  if (UnsyncedData.size() < 2) return false;

  VelocityData front_data = UnsyncedData.at(0);
  VelocityData back_data = UnsyncedData.at(1);
  VelocityData synced_data;

  double front_scale =
      (back_data.time - sync_time) / (back_data.time - front_data.time);
  double back_scale =
      (sync_time - front_data.time) / (back_data.time - front_data.time);
  synced_data.time = sync_time;
  synced_data.linear_velocity.x = front_data.linear_velocity.x * front_scale +
                                  back_data.linear_velocity.x * back_scale;
  synced_data.linear_velocity.y = front_data.linear_velocity.y * front_scale +
                                  back_data.linear_velocity.y * back_scale;
  synced_data.linear_velocity.z = front_data.linear_velocity.z * front_scale +
                                  back_data.linear_velocity.z * back_scale;
  synced_data.angular_velocity.x = front_data.angular_velocity.x * front_scale +
                                   back_data.angular_velocity.x * back_scale;
  synced_data.angular_velocity.y = front_data.angular_velocity.y * front_scale +
                                   back_data.angular_velocity.y * back_scale;
  synced_data.angular_velocity.z = front_data.angular_velocity.z * front_scale +
                                   back_data.angular_velocity.z * back_scale;

  SyncedData.push_back(synced_data);

  return true;
}

void VelocityData::TransformCoordinate(const Eigen::Matrix4f& lidar_to_imu) {
  Eigen::Matrix4d matrix = lidar_to_imu.cast<double>();
  Eigen::Matrix3d rotation = matrix.block<3, 3>(0, 0);
  Eigen::Vector3d w(angular_velocity.x, angular_velocity.y, angular_velocity.z);
  Eigen::Vector3d v(linear_velocity.x, linear_velocity.y, linear_velocity.z);
  //    w,v 乘上旋转矩阵T_lidar_to_imu
  w = rotation * w;
  v = rotation * v;
  Eigen::Vector3d trans(matrix(0, 3), matrix(1, 3), matrix(2, 3));
  Eigen::Vector3d delta_v;
}

}  // namespace lidar_localization