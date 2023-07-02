/*
 * @Description: 前端里程计算法
 * @Author: Ren Qian
 * @Date: 2020-02-04 18:53:06
 */
#include "front_end/front_end.hh"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include "glog/logging.h"

#include <boost/filesystem.hpp>
#include <chrono>
#include "global_defination/global_defination.h"

namespace lh {
FrontEnd::FrontEnd()
    : local_map_ptr_(new PointCloud()),
      global_map_ptr_(new PointCloud()),
      result_cloud_ptr_(new PointCloud()) {
  // 给个默认参数，以免类的使用者在匹配之前忘了设置参数
  std::string config_file_path =
      WORK_SPACE_PATH + "/config/front_end/config.yaml";
  YAML::Node config_node = YAML::LoadFile(config_file_path);

  std::string registration_method =
      config_node["registration_method"].as<std::string>();
  LOG(INFO) << "点云匹配方式为：" << registration_method;

  if (registration_method == "NDT") {
    registration_ptr_ =
        std::make_shared<NDTRegistration>(config_node[registration_method]);
  } else {
    LOG(ERROR) << "没找到与 " << registration_method
               << " 相对应的点云匹配方式!";
    // return false;
  }
  InitDataPath(config_node);
  InitFilter("local_map", local_map_filter_ptr_, config_node);
  InitFilter("frame", frame_filter_ptr_, config_node);
  InitFilter("display", display_filter_ptr_, config_node);
}

bool FrontEnd::InitDataPath(const YAML::Node& config_node) {
  data_path_ = config_node["data_path"].as<std::string>();
  if (data_path_ == "./") {
    data_path_ = WORK_SPACE_PATH;
  }
  data_path_ += "/slam_data";

  if (boost::filesystem::is_directory(data_path_)) {
    boost::filesystem::remove_all(data_path_);
  }

  boost::filesystem::create_directory(data_path_);
  if (!boost::filesystem::is_directory(data_path_)) {
    LOG(WARNING) << "文件夹 " << data_path_ << " 未创建成功!";
    return false;
  } else {
    LOG(INFO) << "地图点云存放地址：" << data_path_;
  }

  std::string key_frame_path = data_path_ + "/key_frames";
  boost::filesystem::create_directory(data_path_ + "/key_frames");
  if (!boost::filesystem::is_directory(key_frame_path)) {
    LOG(WARNING) << "文件夹 " << key_frame_path << " 未创建成功!";
    return false;
  } else {
    LOG(INFO) << "关键帧点云存放地址：" << key_frame_path << std::endl
              << std::endl;
  }

  return true;
}

Eigen::Matrix4f FrontEnd::Update(const CloudData& cloud_data) {
  current_frame_.cloud_data.time = cloud_data.time;
  std::vector<int> indices;
  // current_frame_.cloud_data.cloud_ptr_.reset(
  //     new PointCloud(*cloud_data.cloud_ptr_));
  pcl::removeNaNFromPointCloud(*(cloud_data.cloud_ptr_),
                               *(current_frame_.cloud_data.cloud_ptr_),
                               indices);

  PointCloudPtr filtered_cloud_ptr(new PointCloud());
  frame_filter_ptr_->Filter(current_frame_.cloud_data.cloud_ptr_,
                            filtered_cloud_ptr);

  static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
  static Eigen::Matrix4f last_pose = init_pose_;
  static Eigen::Matrix4f predict_pose = init_pose_;
  static Eigen::Matrix4f last_key_frame_pose = init_pose_;

  // 局部地图容器中没有关键帧，代表是第一帧数据
  // 此时把当前帧数据作为第一个关键帧，并更新局部地图容器和全局地图容器
  if (local_map_frames_.size() == 0) {
    current_frame_.pose = init_pose_;
    UpdateNewFrame(current_frame_);
    return current_frame_.pose;
  }

  // 不是第一帧，就正常匹配
  registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose,
                               result_cloud_ptr_, current_frame_.pose);
  // ndt_ptr_->align(*result_cloud_ptr_, predict_pose);
  // current_frame_.pose = ndt_ptr_->getFinalTransformation();

  // 更新相邻两帧的相对运动
  step_pose = last_pose.inverse() * current_frame_.pose;
  predict_pose = current_frame_.pose * step_pose;
  last_pose = current_frame_.pose;

  // 匹配之后根据距离判断是否需要生成新的关键帧，如果需要，则做相应更新
  if (fabs(last_key_frame_pose(0, 3) - current_frame_.pose(0, 3)) +
          fabs(last_key_frame_pose(1, 3) - current_frame_.pose(1, 3)) +
          fabs(last_key_frame_pose(2, 3) - current_frame_.pose(2, 3)) >
      2.0) {
    UpdateNewFrame(current_frame_);
    last_key_frame_pose = current_frame_.pose;
  }

  return current_frame_.pose;
}

bool FrontEnd::SetInitPose(const Eigen::Matrix4f& init_pose) {
  init_pose_ = init_pose;
  return true;
}

bool FrontEnd::SetPredictPose(const Eigen::Matrix4f& predict_pose) {
  predict_pose_ = predict_pose;
  return true;
}

void FrontEnd::UpdateNewFrame(const Frame& new_key_frame) {
  // 把关键帧点云存储到硬盘里，节省内存
  std::string file_path = data_path_ + "/key_frames/key_frame_" +
                          std::to_string(global_map_frames_.size()) + ".pcd";
  pcl::io::savePCDFileBinary(file_path, *new_key_frame.cloud_data.cloud_ptr_);

  Frame key_frame = new_key_frame;
  // 这一步的目的是为了把关键帧的点云保存下来
  // 由于用的是共享指针，所以直接复制只是复制了一个指针而已
  // 此时无论你放多少个关键帧在容器里，这些关键帧点云指针都是指向的同一个点云
  key_frame.cloud_data.cloud_ptr_.reset(
      new PointCloud(*(new_key_frame.cloud_data.cloud_ptr_)));
  PointCloudPtr transformed_cloud_ptr(new PointCloud());

  // 更新局部地图
  local_map_frames_.push_back(key_frame);
  while (local_map_frames_.size() > 20) {
    local_map_frames_.pop_front();
  }
  local_map_ptr_.reset(new PointCloud());
  for (size_t i = 0; i < local_map_frames_.size(); ++i) {
    pcl::transformPointCloud(*(local_map_frames_.at(i).cloud_data.cloud_ptr_),
                             *transformed_cloud_ptr,
                             local_map_frames_.at(i).pose);
    *local_map_ptr_ += *transformed_cloud_ptr;
  }
  has_new_local_map_ = true;

  // 更新ndt匹配的目标点云
  if (local_map_frames_.size() < 10) {
    registration_ptr_->SetInputTarget(local_map_ptr_);
  } else {
    PointCloudPtr filtered_local_map_ptr(new PointCloud());
    // local_map_filter_.setInputCloud(local_map_ptr_);
    // local_map_filter_.filter(*filtered_local_map_ptr);
    local_map_filter_ptr_->Filter(local_map_ptr_, filtered_local_map_ptr);
    registration_ptr_->SetInputTarget(filtered_local_map_ptr);
  }

  // 更新全局地图
  global_map_frames_.push_back(key_frame);
  if (global_map_frames_.size() % 100 != 0) {
    return;
  } else {
    global_map_ptr_.reset(new PointCloud());
    for (size_t i = 0; i < global_map_frames_.size(); ++i) {
      pcl::transformPointCloud(
          *(global_map_frames_.at(i).cloud_data.cloud_ptr_),
          *transformed_cloud_ptr, global_map_frames_.at(i).pose);
      *global_map_ptr_ += *transformed_cloud_ptr;
    }
    has_new_global_map_ = true;
  }
  // 保存所有关键帧信息在容器里
  // 存储之前，点云要先释放，因为已经存到了硬盘里，不释放也达不到节省内存的目的
  key_frame.cloud_data.cloud_ptr_.reset(new PointCloud());
  global_map_frames_.push_back(key_frame);
}

bool FrontEnd::GetNewLocalMap(PointCloudPtr& local_map_ptr) {
  if (has_new_local_map_) {
    // display_filter_.setInputCloud(local_map_ptr_);
    // display_filter_.filter(*local_map_ptr);
    display_filter_ptr_->Filter(local_map_ptr_, local_map_ptr);

    return true;
  }
  return false;
}

bool FrontEnd::GetNewGlobalMap(PointCloudPtr& global_map_ptr) {
  if (has_new_global_map_) {
    // display_filter_.setInputCloud(global_map_ptr_);
    // display_filter_.filter(*global_map_ptr);
    display_filter_ptr_->Filter(global_map_ptr_, global_map_ptr);
    // global_map_ptr_.reset(new PointCloud());
    return true;
  }
  return false;
}

bool FrontEnd::GetCurrentScan(PointCloudPtr& current_scan_ptr) {
  // display_filter_.setInputCloud(result_cloud_ptr_);
  // display_filter_.filter(*current_scan_ptr);
  display_filter_ptr_->Filter(result_cloud_ptr_, current_scan_ptr);

  return true;
}
}  // namespace lh