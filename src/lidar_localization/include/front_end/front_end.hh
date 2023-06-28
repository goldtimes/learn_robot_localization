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

// 关键帧作为地图，我们要保存地图的位姿和点云数据

class FrontEnd {
 public:
  struct Frame {
   public:
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    CloudData cloud_data_;
  };

  FrontEnd();
  bool InitWithConfig();
  bool Update(const CloudData&, Eigen::Matrix4f& cloud_pose);
  bool setInitPose(const Eigen::Matrix4f& init_pose);
  bool saveMap();
  bool getNewLocalMap(PointCloudPtr& local_map_ptr);
  bool getNewGlobalMap(PointCloudPtr& global_map_ptr);
  bool getCurrentScan(PointCloudPtr& current_scan_ptr);

 private:
  bool initParam(const YAML::Node& config_node);
  bool initDataPath(const YAML::Node& config_node);
  bool initRegistration(
      std::shared_ptr<RegistrationInterface>& registration_ptr,
      const YAML::Node& config_node);
  bool initFilter(std::string filter_use,
                  std::shared_ptr<CloudFilterInterface>& filter_ptr,
                  const YAML::Node& node);
  void updateNewFrame(const Frame& new_key_frame);

 private:
  std::string data_path = "";
  std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
  std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
  std::shared_ptr<CloudFilterInterface> display_filter_ptr_;
  std::shared_ptr<RegistrationInterface> registration_ptr_;

  std::deque<Frame> local_map_frames_;
  std::deque<Frame> global_map_frames_;

  bool has_new_local_map = false;
  bool has_new_global_map = false;

  PointCloudPtr local_map_ptr_;
  PointCloudPtr global_map_ptr_;
  PointCloudPtr match_result_cloud_ptr_;

  Frame current_frame_;

  float key_frame_distance_ = 2.0;
  int local_frame_num_ = 20;
};
}  // namespace lh