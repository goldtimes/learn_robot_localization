/*
 * @Description: front end 任务管理， 放在类里使代码更清晰
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:31:22
 */
#ifndef LIDAR_LOCALIZATION_FRONT_END_FRONT_END_FLOW_HPP_
#define LIDAR_LOCALIZATION_FRONT_END_FRONT_END_FLOW_HPP_

#include <ros/ros.h>

#include "lidar_localization/lidar_undistortion/lidar_undstortion.hh"
#include "lidar_localization/mapping/front_end/front_end.hpp"
#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/odometry_publisher.hpp"
#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/subscriber/gnss_subscriber.hpp"
#include "lidar_localization/subscriber/imu_subscriber.hpp"
#include "lidar_localization/subscriber/velocity_subscriber.hpp"
#include "lidar_localization/tf_listener/tf_listener.hpp"

namespace lidar_localization {
class FrontEndFlow {
 public:
  FrontEndFlow(ros::NodeHandle& nh);

  bool Run();

 private:
  bool ReadData();

  bool HasData();
  bool ValidData();
  bool UpdateLaserOdometry();
  bool PublishData();

 private:
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
  std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
  std::shared_ptr<FrontEnd> front_end_ptr_;

  std::deque<CloudData> cloud_data_buff_;
  IMUData current_imu_data_;
  Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();
};
}  // namespace lidar_localization

#endif