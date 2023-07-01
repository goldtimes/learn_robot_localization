#include "front_end/front_end_flow.hh"
#include <chrono>
#include "glog/logging.h"

namespace lh {
FrontEndFlow::FrontEndFlow(ros::NodeHandle& nh) {
  cloud_sub_ptr_ =
      std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);
  imu_sub_ptr_ = std::make_shared<IMUSub>(nh, "/kitti/oxts/imu", 1000000);
  gnss_sub_ptr_ =
      std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);
  lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh, "velo_link", "imu_link");

  cloud_pub_ptr_ =
      std::make_shared<CloudPublisher>(nh, "current_scan", 100, "/map");
  local_map_pub_ptr_ =
      std::make_shared<CloudPublisher>(nh, "local_map", 100, "/map");
  global_map_pub_ptr_ =
      std::make_shared<CloudPublisher>(nh, "global_map", 100, "/map");
  laser_odom_pub_ptr_ =
      std::make_shared<OdomPublisher>(nh, "laser_odom", "map", "lidar", 100);
  gnss_pub_ptr_ =
      std::make_shared<OdomPublisher>(nh, "gnss", "map", "lidar", 100);

  front_end_ptr_ = std::make_shared<FrontEnd>();

  local_map_ptr_.reset(new PointCloud());
  global_map_ptr_.reset(new PointCloud());
  current_scan_ptr_.reset(new PointCloud());
}

bool FrontEndFlow::Run() {
  ReadData();

  if (!InitCalibration()) return false;

  if (!InitGNSS()) return false;

  while (HasData()) {
    if (!ValidData()) continue;
    UpdateGNSSOdometry();
    std::chrono::steady_clock::time_point start_time =
        std::chrono::steady_clock::now();
    if (UpdateLaserOdometry()) {
      std::chrono::steady_clock::time_point end_time =
          std::chrono::steady_clock::now();
      auto time_used =
          std::chrono::duration_cast<std::chrono::duration<double>>(end_time -
                                                                    start_time);
      std::cout << "time_used: " << time_used.count() << std::endl;
      PublishData();
    }
  }

  return true;
}

bool FrontEndFlow::ReadData() {
  cloud_sub_ptr_->ParseData(cloud_data_buff_);
  imu_sub_ptr_->ParseData(imu_data_buff_);
  gnss_sub_ptr_->ParseData(gnss_data_buff_);

  return true;
}

bool FrontEndFlow::InitCalibration() {
  static bool calibration_received = false;
  if (!calibration_received) {
    if (lidar_to_imu_ptr_->LookupData(lidar_to_imu_)) {
      calibration_received = true;
    }
  }

  return calibration_received;
}

bool FrontEndFlow::InitGNSS() {
  static bool gnss_inited = false;
  if (!gnss_inited && gnss_data_buff_.size() > 0) {
    GNSSData gnss_data = gnss_data_buff_.front();
    gnss_data.InitOriginPosition();
    gnss_inited = true;
  }

  return gnss_inited;
}

bool FrontEndFlow::HasData() {
  if (cloud_data_buff_.size() == 0) return false;
  if (imu_data_buff_.size() == 0) return false;
  if (gnss_data_buff_.size() == 0) return false;

  return true;
}

bool FrontEndFlow::ValidData() {
  current_cloud_data_ = cloud_data_buff_.front();
  current_imu_data_ = imu_data_buff_.front();
  current_gnss_data_ = gnss_data_buff_.front();

  double d_time = current_cloud_data_.time - current_imu_data_.time;
  if (d_time < -0.05) {
    cloud_data_buff_.pop_front();
    return false;
  }

  if (d_time > 0.05) {
    imu_data_buff_.pop_front();
    gnss_data_buff_.pop_front();
    return false;
  }

  cloud_data_buff_.pop_front();
  imu_data_buff_.pop_front();
  gnss_data_buff_.pop_front();

  return true;
}

bool FrontEndFlow::UpdateGNSSOdometry() {
  gnss_odometry_ = Eigen::Matrix4f::Identity();

  current_gnss_data_.UpdateXYZ();
  gnss_odometry_(0, 3) = current_gnss_data_.local_E;
  gnss_odometry_(1, 3) = current_gnss_data_.local_N;
  gnss_odometry_(2, 3) = current_gnss_data_.local_U;
  gnss_odometry_.block<3, 3>(0, 0) = current_imu_data_.getOrientationMatrix();
  gnss_odometry_ *= lidar_to_imu_;
  gnss_pub_ptr_->Publish(gnss_odometry_);
  front_end_ptr_->SetInitPose(gnss_odometry_);

  return true;
}

bool FrontEndFlow::UpdateLaserOdometry() {
  static bool front_end_pose_inited = false;
  if (!front_end_pose_inited) {
    front_end_pose_inited = true;
    front_end_ptr_->SetInitPose(gnss_odometry_);
    laser_odometry_ = gnss_odometry_;
    return true;
  }

  laser_odometry_ = Eigen::Matrix4f::Identity();
  // if (front_end_ptr_->Update(current_cloud_data_, laser_odometry_))
  //   return true;
  // else
  //   return false;
}

bool FrontEndFlow::PublishData() {
  laser_odom_pub_ptr_->Publish(laser_odometry_);

  front_end_ptr_->GetCurrentScan(current_scan_ptr_);
  cloud_pub_ptr_->Publish(current_scan_ptr_);

  if (front_end_ptr_->GetNewLocalMap(local_map_ptr_))
    local_map_pub_ptr_->Publish(local_map_ptr_);

  return true;
}

// bool FrontEndFlow::SaveMap() { return front_end_ptr_->SaveMap(); }

bool FrontEndFlow::PublishGlobalMap() {
  if (front_end_ptr_->GetNewGlobalMap(global_map_ptr_)) {
    global_map_pub_ptr_->Publish(global_map_ptr_);
    global_map_ptr_.reset(new PointCloud());
  }
  return true;
}
}  // namespace lh