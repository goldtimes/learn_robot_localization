#pragma once

#include <ros/ros.h>
#include <deque>
// sub
#include "lidar_localization/subscriber/cloud_subscriber.hpp"
// 前端模块提供的key_frames 和 odometrys
#include "lidar_localization/subscriber/key_frame_subsriber.hpp"
#include "lidar_localization/subscriber/key_frames_subsriber.hpp"
#include "lidar_localization/subscriber/odometry_subscriber.hpp"
// pub
#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/odometry_publisher.hpp"

#include "lidar_localization/mapping/viewer/viewer.hh"

namespace lidar_localization {
class ViewerFlow {
 public:
  ViewerFlow(ros::NodeHandle nh);
  bool Run();

  bool SaveMap();

 private:
  bool ReadData();
  bool HasData();
  bool ValidData();
  bool UpdateViewer();
  bool PublishData();

 private:
  //  sub
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
  std::shared_ptr<OdometrySubscriber> transformed_odom_sub_ptr_;
  std::shared_ptr<KeyFrameSubscriber> key_frame_sub_ptr_;
  std::shared_ptr<KeyFrameSubscriber> optimized_key_frames_sub_ptr_;
  // pub
  std::shared_ptr<OdometryPublisher> optimized_odom_pub_ptr_;
  std::shared_ptr<CloudPublisher> current_scan_pub_ptr_;
  std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
  std::shared_ptr<CloudPublisher> local_map_pub_ptr_;
  // viewer
  std::shared_ptr<Viewer> viewer_ptr_;
  //  datas
  std::deque<CloudData> cloud_data_buff_;
  std::deque<PoseData> cloud_data_buff_;
  std::deque<KeyFrame> key_frame_buff_;
  std::deque<KeyFrame> optimized_key_frames_;
  std::deque<KeyFrame> all_key_frames_;

  CloudData current_cloud_data_;
  PoseData current_transformed_odom_;
};
}  // namespace lidar_localization