#pragma once

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <deque>
#include "lidar_localization/sensor_data/pose_data.hpp"

namespace lidar_localization {
class OdometrySubscriber {
 public:
  OdometrySubscriber(ros::NodeHandle& nh, std::string topic_name,
                     size_t buff_size);
  OdometrySubscriber() = default;
  void ParseData(std::deque<PoseData>& deque_pose_data);

 private:
  void msg_callback(const nav_msgs::OdometryConstPtr& odom_msg_ptr);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;

  std::deque<PoseData> new_pose_data_;
};
}  // namespace lidar_localization