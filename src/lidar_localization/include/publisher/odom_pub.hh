#pragma once

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <string>
namespace lh {
class OdomPublisher {
 public:
  OdomPublisher() = default;
  OdomPublisher(const ros::NodeHandle& nh_, const std::string& topic_name,
                const std::string& parent_frame_name, const std::string& child_frame_name,
                int topic_size);
  void Publish(const Eigen::Matrix4d& transform);

 private:
  ros::NodeHandle nh_;
  ros::Publisher odom_pub_;
  nav_msgs::Odometry odom_;
};
}  // namespace lh