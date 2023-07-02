/*
 * @Description: NDT 匹配模块
 * @Author: Ren Qian
 * @Date: 2020-02-08 21:46:57
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_NDT_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_NDT_REGISTRATION_HPP_

#include <pcl/registration/ndt.h>
#include "models/registration/registration_interface.hpp"

namespace lh {
class NDTRegistration : public RegistrationInterface {
 public:
  NDTRegistration(const YAML::Node& node);
  NDTRegistration(float res, float step_size, float trans_eps, int max_iter);

  bool SetInputTarget(const PointCloudPtr& input_target) override;
  bool ScanMatch(const PointCloudPtr& input_source,
                 const Eigen::Matrix4f& predict_pose,
                 PointCloudPtr& result_cloud_ptr,
                 Eigen::Matrix4f& result_pose) override;

 private:
  bool SetRegistrationParam(float res, float step_size, float trans_eps,
                            int max_iter);

 private:
  pcl::NormalDistributionsTransform<PointXYZ, PointXYZ>::Ptr ndt_ptr_;
};
}  // namespace lh

#endif