#pragma once

#include <Eigen/Dense>
#include <deque>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/registration/ndt.h>
#include "models/cloud_filter/voxel_filter.hpp"
#include "models/registration/ndt_registration.hpp"

#include "glog/logging.h"
#include "sensor_data/cloud_data.hpp"

namespace lh {
class FrontEnd {
 public:
  class Frame {
   public:
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
  bool InitDataPath(const YAML::Node& config_node);

 private:
  // pcl::VoxelGrid<PointXYZ> cloud_filter_;
  // pcl::VoxelGrid<PointXYZ> local_map_filter_;
  // pcl::VoxelGrid<PointXYZ> display_filter_;
  std::string data_path_;
  std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
  std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
  std::shared_ptr<CloudFilterInterface> display_filter_ptr_;

  bool InitFilter(std::string filter_user,
                  std::shared_ptr<CloudFilterInterface>& filter_ptr,
                  const YAML::Node& config_node) {
    std::string filter_mothod =
        config_node[filter_user + "_filter"].as<std::string>();
    LOG(INFO) << filter_user << "选择的滤波方法为：" << filter_mothod;

    if (filter_mothod == "voxel_filter") {
      filter_ptr = std::make_shared<VoxelFilter>(
          config_node[filter_mothod][filter_user]);
    } else {
      LOG(ERROR) << "没有为 " << filter_user << " 找到与 " << filter_mothod
                 << " 相对应的滤波方法!";
      return false;
    }

    return true;
  }
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
  // pcl::NormalDistributionsTransform<PointXYZ, PointXYZ>::Ptr ndt_ptr_;
  std::shared_ptr<RegistrationInterface> registration_ptr_;
};
}  // namespace lh
