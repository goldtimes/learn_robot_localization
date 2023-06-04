#include "publisher/odom_pub.hh"

namespace lh {
OdomPublisher::OdomPublisher(const ros::NodeHandle& nh,
                             const std::string& topic_name,
                             const std::string& parent_frame_name,
                             const std::string& child_frame_name,
                             int topic_size)
    : nh_(nh) {
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>(topic_name, topic_size);
  odom_.header.frame_id = parent_frame_name;
  odom_.child_frame_id = child_frame_name;
}

void OdomPublisher::Publish(const Eigen::Matrix4f& transform) {
  odom_.header.stamp = ros::Time::now();
  odom_.pose.pose.position.x = transform(0, 3);
  odom_.pose.pose.position.y = transform(1, 3);
  odom_.pose.pose.position.z = transform(2, 3);

  Eigen::Quaternionf q;
  q = transform.block<3, 3>(0, 0);
  q.normalize();
  odom_.pose.pose.orientation.x = q.x();
  odom_.pose.pose.orientation.y = q.y();
  odom_.pose.pose.orientation.z = q.z();
  odom_.pose.pose.orientation.w = q.w();
  odom_pub_.publish(odom_);
}
}  // namespace lh