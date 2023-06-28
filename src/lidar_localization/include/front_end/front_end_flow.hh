#pragma once

#include <memory>
#include "front_end/front_end.hh"
#include "publisher/cloud_pub.hh"
#include "publisher/odom_pub.hh"
#include "subscriber/cloud_sub.hh"
#include "subscriber/gnss_sub.hh"
#include "subscriber/imu_sub.hh"
#include "tf/tf_listener.hh"

// slam 前端的处理流程类
namespace lh {
class FrontEndFlow {
 public:
  FrontEndFlow(ros::NodeHandle& nh);
  bool Run();
  bool SaveMap();
  bool PublishGlobalMap();

 private:
  bool ReadData();
  // lidar_to_imu的坐标变换
  bool InitCalibration();
  bool InitGNSS();
  bool HasData();
  bool ValidData();
  bool UpdateGNSSOdometry();
  bool UpdateLaserOdometry();
  bool PublishData();

 private:
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
  std::shared_ptr<IMUSub> imu_sub_ptr_;
  std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;
  std::shared_ptr<TFListener> lidar_to_imu_ptr_;
  std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
  std::shared_ptr<CloudPublisher> local_map_pub_ptr_;
  std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
  std::shared_ptr<OdomPublisher> lidar_odom_pub_ptr_;
  std::shared_ptr<OdomPublisher> gnss_odom_pub_ptr_;
  std::shared_ptr<FrontEnd> front_end_ptr_;

  std::deque<CloudData> cloud_data_buff_;
  std::deque<ImuData> imu_data_buff_;
  std::deque<GNSSData> gnss_data_buff_;
  // 初始化姿态
  Eigen::Matrix4f lidar_to_imu = Eigen::Matrix4f::Identity();
  CloudData current_cloud_data_;
  ImuData current_imu_data_;
  GNSSData current_gnss_data_;

  PointCloudPtr local_map_ptr_;
  PointCloudPtr global_map_ptr_;
  PointCloudPtr current_scan_ptr_;

  Eigen::Matrix4f gnss_odom_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f lidar_odom_ = Eigen::Matrix4f::Identity();
};
}  // namespace lh