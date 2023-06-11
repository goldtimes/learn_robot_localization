#pragma once

#include <Eigen/Dense>
#include <deque>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

#include "sensor_data/cloud_data.hpp"

namespace lh {

// 关键帧作为地图，我们要保存地图的位姿和点云数据
class Frame {
 public:
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  CloudData cloud_data_;
};

class FrontEnd {
 public:
  FrontEnd();
  // 更新点云
  Eigen::Matrix4f update(const CloudData& cloud_data);

  bool setInitPose(const Eigen::Matrix4f& init_pose);
  bool setPredictPose(const Eigen::Matrix4f& predict_pose);

  bool getNewLocalMap(PointCloudPtr& local_map_ptr);
  bool getNewGlobalMap(PointCloudPtr& global_map_ptr);
  bool getCurrentScan(PointCloudPtr& current_scan_ptr);

 private:
  void updateNewFrame(const Frame& new_key_frame);

 private:
  // 滤波器
  pcl::VoxelGrid<PointXYZ> cloud_filter_;
  // 地图的滤波器
  pcl::VoxelGrid<PointXYZ> local_map_filter_;
  pcl::VoxelGrid<PointXYZ> display_filter_;
  // 匹配
  pcl::NormalDistributionsTransform<PointXYZ, PointXYZ>::Ptr ndt_ptr_;
  std::deque<Frame> local_map_frames_;
  std::deque<Frame> global_map_frames_;

  bool has_new_local_map = false;
  bool has_new_global_map = false;

  PointCloudPtr local_map_ptr_;
  PointCloudPtr global_map_ptr_;
  PointCloudPtr match_result_cloud_ptr_;

  Frame current_frame_;

  // 预测的初始位姿
  Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
  // 匹配出来的位姿
  Eigen::Matrix4f predict_pose_ = Eigen::Matrix4f::Identity();
};
}  // namespace lh
