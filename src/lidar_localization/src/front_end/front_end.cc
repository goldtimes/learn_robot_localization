#include "front_end/front_end.hh"
#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <cmath>
#include <vector>

namespace lh {
FrontEnd::FrontEnd()
    : ndt_ptr_(new pcl::NormalDistributionsTransform<PointXYZ, PointXYZ>()),
      local_map_ptr_(new PointCloud()),
      global_map_ptr_(new PointCloud()),
      match_result_cloud_ptr_(new PointCloud()) {
  // 过滤的参数
  cloud_filter_.setLeafSize(1.3, 1.3, 1.3);
  local_map_filter_.setLeafSize(0.6, 0.6, 0.6);
  display_filter_.setLeafSize(0.5, 0.5, 0.5);
  ndt_ptr_->setResolution(1.0);
  ndt_ptr_->setStepSize(0.1);
  ndt_ptr_->setTransformationEpsilon(0.1);
  ndt_ptr_->setMaximumIterations(30);
}
bool FrontEnd::setInitPose(const Eigen::Matrix4f& init_pose) {
  init_pose_ = init_pose;
  return true;
}

bool FrontEnd::setPredictPose(const Eigen::Matrix4f& predict_pose) {
  predict_pose_ = predict_pose;
  return true;
}

Eigen::Matrix4f FrontEnd::update(const CloudData& cloud_data) {
  // 当前frame
  current_frame_.cloud_data_.time = cloud_data.time;
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr_,
                               *current_frame_.cloud_data_.cloud_ptr_, indices);
  // 滤波后的点云
  PointCloudPtr filtered_cloud_ptr(new PointCloud());
  cloud_filter_.setInputCloud(current_frame_.cloud_data_.cloud_ptr_);
  cloud_filter_.filter(*filtered_cloud_ptr);

  static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
  static Eigen::Matrix4f last_post = init_pose_;
  static Eigen::Matrix4f predict_post = init_pose_;
  static Eigen::Matrix4f last_key_frame_pose = init_pose_;
  // 容器为空,第一帧的数据
  if (local_map_frames_.size() == 0) {
    current_frame_.pose = init_pose_;
    // 更新局部地图容器和全局地图容器
    updateNewFrame(current_frame_);
    return current_frame_.pose;
  }
  // 不是第一帧的数据

  ndt_ptr_->setInputCloud(filtered_cloud_ptr);
  // 点云local_map_ptr去匹配
  ndt_ptr_->align(*match_result_cloud_ptr_, predict_post);
  //   std::cout << "match_result_cloud_ptr_: " <<
  //   match_result_cloud_ptr_->size()
  //             << std::endl;
  current_frame_.pose = ndt_ptr_->getFinalTransformation();
  // 更新相邻两帧的相对运动
  step_pose = last_post.inverse() * current_frame_.pose;
  predict_post = current_frame_.pose * step_pose;
  last_post = current_frame_.pose;
  // 判断相邻两帧的距离 > 0.66
  if ((fabs(last_key_frame_pose(0, 3) - current_frame_.pose(0, 3)) +
       fabs(last_key_frame_pose(1, 3) - current_frame_.pose(1, 3)) +
       fabs(last_key_frame_pose(2, 3) - current_frame_.pose(2, 3))) > 2.0) {
    updateNewFrame(current_frame_);
    last_key_frame_pose = current_frame_.pose;
  }
  return current_frame_.pose;
}

void FrontEnd::updateNewFrame(const Frame& new_key_frame) {
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
  for (int i = 0; i < local_map_frames_.size(); ++i) {
    // 将激光雷达坐标系下的点云转换成map坐标系下的点云
    pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data_.cloud_ptr_,
                             *transformed_cloud_ptr,
                             local_map_frames_.at(i).pose);
    // local_map 不是一帧，而是二十帧的点云集合
    *local_map_ptr_ += *transformed_cloud_ptr;
  }
  has_new_local_map = true;
  // 更新ndt匹配的目标点云
  if (local_map_frames_.size() < 10) {
    ndt_ptr_->setInputTarget(local_map_ptr_);
  } else {
    // 大于10帧，经过过滤后在匹配
    PointCloudPtr filter_local_map_ptr(new PointCloud());
    local_map_filter_.setInputCloud(local_map_ptr_);
    local_map_filter_.filter(*filter_local_map_ptr);
    ndt_ptr_->setInputTarget(filter_local_map_ptr);
  }
  // 更新全局地图
  global_map_frames_.push_back(key_frame);
  if (global_map_frames_.size() % 100 != 0) {
    return;
  } else {
    global_map_ptr_.reset(new PointCloud());
    for (int i = 0; i < global_map_frames_.size(); ++i) {
      pcl::transformPointCloud(*global_map_frames_.at(i).cloud_data_.cloud_ptr_,
                               *transformed_cloud_ptr,
                               global_map_frames_.at(i).pose);
      *global_map_ptr_ += *transformed_cloud_ptr;
    }
    has_new_global_map = true;
  }
}

bool FrontEnd::getNewLocalMap(PointCloudPtr& local_map_ptr) {
  if (has_new_local_map) {
    display_filter_.setInputCloud(local_map_ptr_);
    display_filter_.filter(*local_map_ptr);
    return true;
  }
  return false;
}

bool FrontEnd::getNewGlobalMap(PointCloudPtr& global_map_ptr) {
  if (has_new_global_map) {
    display_filter_.setInputCloud(global_map_ptr_);
    display_filter_.filter(*global_map_ptr);
    return true;
  }
  return false;
}

bool FrontEnd::getCurrentScan(PointCloudPtr& current_scan_ptr) {
  display_filter_.setInputCloud(match_result_cloud_ptr_);
  display_filter_.filter(*current_scan_ptr);
  return true;
}
}  // namespace lh