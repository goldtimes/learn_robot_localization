#pragma once

#include <Eigen/Dense>
#include <deque>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

#include <yaml-cpp/yaml.h>
#include "models/cloud_filter/cloud_filter_interface.hh"
#include "models/cloud_filter/voxel_filter.hh"
#include "models/registrations/ndt_registration.hh"
#include "models/registrations/registration_interface.hh"
#include "sensor_data/cloud_data.hpp"

namespace lh {
class FrontEnd {
 public:
  struct Frame {
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    CloudData cloud_data;
  };

 public:
  FrontEnd();

  Eigen::Matrix4f Update(const CloudData& cloud_data);
  bool SetInitPose(const Eigen::Matrix4f& init_pose);
  bool SetPredictPose(const Eigen::Matrix4f& predict_pose);

  bool GetNewLocalMap(PointCloudPtr& local_map_ptr);
  bool GetNewGlobalMap(PointCloudPtr& global_map_ptr);
  bool GetCurrentScan(PointCloudPtr& current_scan_ptr);

 private:
  void UpdateNewFrame(const Frame& new_key_frame);

 private:
  std::string data_path_ = "";

  // std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
  // std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
  // std::shared_ptr<CloudFilterInterface> display_filter_ptr_;
  // std::shared_ptr<RegistrationInterface> registration_ptr_;

  pcl::VoxelGrid<PointXYZ> cloud_filter_;
  pcl::VoxelGrid<PointXYZ> local_map_filter_;
  pcl::VoxelGrid<PointXYZ> display_filter_;

  std::deque<Frame> local_map_frames_;
  std::deque<Frame> global_map_frames_;

  bool has_new_local_map_ = false;
  bool has_new_global_map_ = false;
  PointCloudPtr local_map_ptr_;
  PointCloudPtr global_map_ptr_;
  PointCloudPtr result_cloud_ptr_;
  Frame current_frame_;

  Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f predict_pose_ = Eigen::Matrix4f::Identity();
  pcl::NormalDistributionsTransform<PointXYZ, PointXYZ>::Ptr ndt_ptr_;

  float key_frame_distance_ = 2.0;
  int local_frame_num_ = 20;
};
}  // namespace lh
