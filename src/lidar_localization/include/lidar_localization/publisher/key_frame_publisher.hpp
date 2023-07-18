#pragma once
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <string>

#include "lidar_localization/sensor_data/key_frame.hpp"

namespace lidar_localization {
class KeyFramePublisher {
 public:
  KeyFramePublisher(ros::NodeHandle& nh, std::string topic_name,
                    std::string frame_id, int buff_size);
  KeyFramePublisher() = default;

  void Publish(KeyFrame& key_frame);

  bool HasSubscribers();

 private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  std::string frame_id_ = "";
};
}  // namespace lidar_localization