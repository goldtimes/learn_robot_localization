/*
 * @Description: 点云匹配模块的基类
 * @Author: Ren Qian
 * @Date: 2020-02-08 21:25:11
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_INTERFACE_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_INTERFACE_HPP_

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include "sensor_data/cloud_data.hpp"

namespace lh {
class RegistrationInterface {
 public:
  virtual ~RegistrationInterface() = default;

  virtual bool SetInputTarget(const PointCloudPtr& input_target) = 0;
  virtual bool ScanMatch(const PointCloudPtr& input_source,
                         const Eigen::Matrix4f& predict_pose,
                         PointCloudPtr& result_cloud_ptr,
                         Eigen::Matrix4f& result_pose) = 0;
};
}  // namespace lh

#endif