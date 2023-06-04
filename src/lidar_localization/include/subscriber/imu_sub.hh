#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <deque>
#include "sensor_data/imu_data.hpp"

namespace lh {
class IMUSub {
 public:
  IMUSub(ros::NodeHandle& nh, const std::string& topic_name, size_t topic_size);
  IMUSub() = default;
  void ParseData(std::deque<ImuData>& deque_imu_data);

 private:
  void imu_callback(const sensor_msgs::ImuConstPtr& imu_msg);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber imu_sub_;
  std::deque<ImuData> new_imu_data_;
};
}  // namespace lh