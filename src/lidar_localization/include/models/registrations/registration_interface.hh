#pragma once
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include "sensor_data/cloud_data.hpp"

namespace lh {
class RegistrationInterface {
 public:
  virtual ~RegistrationInterface() = default;
  virtual bool SetInputTraget(const PointCloudPtr& input_target) = 0;
  virtual bool scanMatch(const PointCloudPtr& input_source,
                         const Eigen::Matrix4f& predict_pose,
                         PointCloudPtr& result_cloud_ptr,
                         Eigen::Matrix4f result_pose) = 0;
};
}  // namespace lh