/*
 * @Description: 点云滤波模块的接口
 * @Author: Ren Qian
 * @Date: 2020-02-09 19:29:50
 */
#ifndef LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_HPP_
#define LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_HPP_

#include <yaml-cpp/yaml.h>
#include "sensor_data/cloud_data.hpp"

namespace lh {
class CloudFilterInterface {
 public:
  virtual ~CloudFilterInterface() = default;

  virtual bool Filter(const PointCloudPtr& input_cloud_ptr,
                      PointCloudPtr& filtered_cloud_ptr) = 0;
};
}  // namespace lh

#endif