#pragma once
#include "lidar_localization/sensor_data/imu_data.hpp"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

namespace lidar_localization {
class IMUPublisher {
 public:
  IMUPublisher(ros::NodeHandle& nh, std::string topic_name, size_t buff_size,
               std::string frame_id);
  IMUPublisher() = default;

  void Publish(IMUData imu_data);

  bool HasSubscribers();

 private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  std::string frame_id_;
};
}  // namespace lidar_localization