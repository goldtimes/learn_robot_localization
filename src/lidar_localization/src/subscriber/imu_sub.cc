#include "subscriber/imu_sub.hh"
#include "glog/logging.h"

namespace lh {
IMUSub::IMUSub(ros::NodeHandle& nh, const std::string& topic_name, size_t buff_size) : nh_(nh) {
  imu_sub_ = nh_.subscribe(topic_name, buff_size, &IMUSub::imu_callback, this);
}
void IMUSub::imu_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr) {
  ImuData imu_data;
  imu_data.time = imu_msg_ptr->header.stamp.toSec();

  imu_data.linear_acc_.x = imu_msg_ptr->linear_acceleration.x;
  imu_data.linear_acc_.y = imu_msg_ptr->linear_acceleration.y;
  imu_data.linear_acc_.z = imu_msg_ptr->linear_acceleration.z;

  imu_data.angular_vel_.x = imu_msg_ptr->angular_velocity.x;
  imu_data.angular_vel_.y = imu_msg_ptr->angular_velocity.y;
  imu_data.angular_vel_.z = imu_msg_ptr->angular_velocity.z;

  imu_data.orientation_.x = imu_msg_ptr->orientation.x;
  imu_data.orientation_.y = imu_msg_ptr->orientation.y;
  imu_data.orientation_.z = imu_msg_ptr->orientation.z;
  imu_data.orientation_.w = imu_msg_ptr->orientation.w;

  new_imu_data_.push_back(imu_data);
}
void IMUSub::ParseData(std::deque<ImuData>& imu_data_buff) {
  if (new_imu_data_.size() > 0) {
    imu_data_buff.insert(imu_data_buff.end(), new_imu_data_.begin(), new_imu_data_.end());
    new_imu_data_.clear();
  }
}
}  // namespace lh