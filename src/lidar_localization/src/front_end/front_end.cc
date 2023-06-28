#include "front_end/front_end.hh"
#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <cmath>
#include <string>
#include <vector>
#include "global_defination/global_defination.h"

namespace lh {
FrontEnd::FrontEnd()
    : local_map_ptr_(new PointCloud()),
      global_map_ptr_(new PointCloud()),
      match_result_cloud_ptr_(new PointCloud()) {
  InitWithConfig();
}
bool FrontEnd::InitWithConfig() {
  std::string config_file_path =
      WORK_SPACE_PATH + "/config/front_end/config.yaml";
  YAML::Node config_node = YAML::LoadFile(config_file_path);
  initDataPath(config_node);
  initRegistration(registration_ptr_, config_node);
  initFilter("local_map", local_map_filter_ptr_, config_node);
  initFilter("frame", frame_filter_ptr_, config_node);
  initFilter("display", display_filter_ptr_, config_node);
}

bool FrontEnd::initParam(const YAML::Node& config_node) {
  key_frame_distance_ = config_node["key_frame_distance"].as<float>();
  local_frame_num_ = config_node["local_frame_num"].as<int>();

  return true;
}

bool FrontEnd::initDataPath(const YAML::Node& config_node) {
  data_path = config_node["data_path"].as<std::string>();
  if (data_path == "./") {
    data_path = WORK_SPACE_PATH;
  }
  data_path += "/slam_data";
  if (boost::filesystem::is_directory(data_path)) {
    boost::filesystem::remove_all(data_path);
  }

  boost::filesystem::create_directory(data_path);
  if (!boost::filesystem::is_directory(data_path)) {
    LOG(WARNING) << "文件夹 " << data_path << " 未创建成功!";
    return false;
  } else {
    LOG(INFO) << "地图点云存放地址：" << data_path;
  }

  std::string key_frame_path = data_path + "/key_frames";
  boost::filesystem::create_directory(data_path + "/key_frames");
  if (!boost::filesystem::is_directory(key_frame_path)) {
    LOG(WARNING) << "文件夹 " << key_frame_path << " 未创建成功!";
    return false;
  } else {
    LOG(INFO) << "关键帧点云存放地址：" << key_frame_path << std::endl
              << std::endl;
  }
  return true;
}
bool FrontEnd::initRegistration(
    std::shared_ptr<RegistrationInterface>& registration_ptr,
    const YAML::Node& config_node) {
  std::string registration_method =
      config_node["registration_method"].as<std::string>();
  LOG(INFO) << "点云匹配方式为：" << registration_method;
  if (registration_method == "NDT") {
    registration_ptr_ =
        std::make_shared<NDTRegistration>(config_node[registration_method]);
  } else {
    LOG(ERROR) << "没找到与 " << registration_method
               << " 相对应的点云匹配方式!";
    return false;
  }
  return true;
}

bool FrontEnd::initFilter(std::string filter_user,
                          std::shared_ptr<CloudFilterInterface>& filter_ptr,
                          const YAML::Node& config_node) {
  std::string filter_mothod =
      config_node[filter_user + "_filter"].as<std::string>();
  LOG(INFO) << filter_user << "选择的滤波方法为：" << filter_mothod;

  if (filter_mothod == "voxel_filter") {
    filter_ptr =
        std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);
  } else {
    LOG(ERROR) << "没有为 " << filter_user << " 找到与 " << filter_mothod
               << " 相对应的滤波方法!";
    return false;
  }

  return true;
}

bool FrontEnd::setInitPose(const Eigen::Matrix4f& init_pose) {
  init_pose_ = init_pose;
  return true;
}

bool FrontEnd::Update(const CloudData& cloud_data,
                      Eigen::Matrix4f& cloud_pose) {
  // 当前frame
  current_frame_.cloud_data_.time = cloud_data.time;
  std::vector<int> indices;
  // 去除nan的点
  pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr_,
                               *current_frame_.cloud_data_.cloud_ptr_, indices);
  // 滤波后的点云
  PointCloudPtr filtered_cloud_ptr(new PointCloud());
  frame_filter_ptr_->Filter(current_frame_.cloud_data_.cloud_ptr_,
                            filtered_cloud_ptr);
  // 不一定要用static, 记录初始的位置
  static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
  static Eigen::Matrix4f last_pose = init_pose_;
  static Eigen::Matrix4f predict_pose = init_pose_;
  static Eigen::Matrix4f last_key_frame_pose = init_pose_;
  // 容器为空,第一帧的数据
  if (local_map_frames_.size() == 0) {
    // 当前帧的位姿信息
    current_frame_.pose = init_pose_;
    // 更新局部地图容器和全局地图容器
    updateNewFrame(current_frame_);
    cloud_pose = current_frame_.pose;
    return true;
  }

  // 不是第一帧的数据
  registration_ptr_->scanMatch(filtered_cloud_ptr, predict_pose,
                               match_result_cloud_ptr_, current_frame_.pose);
  cloud_pose = current_frame_.pose;
  // 更新相邻两帧的相对运动,增量
  step_pose = last_pose.inverse() * current_frame_.pose;
  // 预测的位姿 = 匹配后的位姿 * 增量?
  predict_pose = current_frame_.pose * step_pose;
  last_pose = current_frame_.pose;
  // 判断相邻两帧的距离 > 0.66
  if ((fabs(last_key_frame_pose(0, 3) - current_frame_.pose(0, 3)) +
       fabs(last_key_frame_pose(1, 3) - current_frame_.pose(1, 3)) +
       fabs(last_key_frame_pose(2, 3) - current_frame_.pose(2, 3))) >
      key_frame_distance_) {
    updateNewFrame(current_frame_);
    last_key_frame_pose = current_frame_.pose;
  }
  return true;
}

bool FrontEnd::updateNewFrame(const Frame& new_key_frame) {
  // 将关键帧点云存储在硬盘中
  std::string file_path = data_path + "/key_frames/key_frame_" +
                          std::to_string(global_map_frames_.size()) + ".pcd";
  pcl::io::savePCDFileBinary(file_path, *new_key_frame.cloud_data_.cloud_ptr_);
  Frame key_frame = new_key_frame;
  key_frame.cloud_data_.cloud_ptr_.reset(
      new PointCloud(*new_key_frame.cloud_data_.cloud_ptr_));
  // 创建变换后的点云
  PointCloudPtr transformed_cloud_ptr(new PointCloud());
  // localmap deque维护的frame个数是20个
  local_map_frames_.push_back(key_frame);
  while (local_map_frames_.size() > 20) {
    local_map_frames_.pop_front();
  }
  local_map_ptr_.reset(new PointCloud());
  // 因为记录的pose是map坐标系下的姿态,所以通过transformPointCloud将lidar下面的点云转换到
  // map坐标系下
  for (int i = 0; i < local_map_frames_.size(); ++i) {
    // 将激光雷达坐标系下的点云转换成map坐标系下的点云
    pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data_.cloud_ptr_,
                             *transformed_cloud_ptr,
                             local_map_frames_.at(i).pose);
    // local_map 不是一帧，而是二十帧的点云集合
    *local_map_ptr_ += *transformed_cloud_ptr;
  }
  has_new_local_map_ = true;
  // 更新ndt匹配的目标点云
  if (local_map_frames_.size() < 10) {
    registration_ptr_->SetInputTraget(local_map_ptr_);
  } else {
    // 大于10帧，经过过滤后在匹配
    PointCloudPtr filter_local_map_ptr(new PointCloud());
    local_map_filter_ptr_->Filter(local_map_ptr_, filter_local_map_ptr);
    registration_ptr_->SetInputTraget(filter_local_map_ptr);
  }
  key_frame.cloud_data_.cloud_ptr_.reset(new PointCloud());
  global_map_frames_.push_back(key_frame);
  return true;
}

bool FrontEnd::saveMap() {
  global_map_ptr_.reset(new PointCloud());

  std::string key_frame_path = "";
  PointCloudPtr key_frame_cloud_ptr(new PointCloud());
  PointCloudPtr transformed_cloud_ptr(new PointCloud());

  for (size_t i = 0; i < global_map_frames_.size(); ++i) {
    key_frame_path =
        data_path + "/key_frames/key_frame_" + std::to_string(i) + ".pcd";
    pcl::io::loadPCDFile(key_frame_path, *key_frame_cloud_ptr);

    pcl::transformPointCloud(*key_frame_cloud_ptr, *transformed_cloud_ptr,
                             global_map_frames_.at(i).pose);
    *global_map_ptr_ += *transformed_cloud_ptr;
  }

  std::string map_file_path = data_path + "/map.pcd";
  pcl::io::savePCDFileBinary(map_file_path, *global_map_ptr_);
  has_new_global_map_ = true;

  return true;
}

bool FrontEnd::getNewLocalMap(PointCloudPtr& local_map_ptr) {
  if (has_new_local_map_) {
    display_filter_ptr_->Filter(local_map_ptr_, local_map_ptr);
    return true;
  }
  return false;
}

bool FrontEnd::getNewGlobalMap(PointCloudPtr& global_map_ptr) {
  if (has_new_global_map_) {
    display_filter_ptr_->Filter(global_map_ptr_, global_map_ptr);
    return true;
  }
  return false;
}

bool FrontEnd::getCurrentScan(PointCloudPtr& current_scan_ptr) {
  display_filter_ptr_->Filter(match_result_cloud_ptr_, current_scan_ptr);
  return true;
}
}  // namespace lh