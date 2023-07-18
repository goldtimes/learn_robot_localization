#pragma once
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <deque>

#include "lidar_localization/sensor_data/key_frame.hpp"

namespace lidar_localization {
class KeyFrameSubscriber {
 public:
  KeyFrameSubscriber(ros::NodeHandle& nh, std::string topic_name,
                     size_t buff_size);
  KeyFrameSubscriber() = default;
  void ParseData(std::deque<KeyFrame>& key_frame_buff);

 private:
  void msg_callback(
      const geometry_msgs::PoseStampedConstPtr& key_frame_msg_ptr);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;

  std::deque<KeyFrame> new_key_frame_;
};
}  // namespace lidar_localization