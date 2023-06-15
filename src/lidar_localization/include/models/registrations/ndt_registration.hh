#pragma once
#include <pcl/registration/ndt.h>
#include "registrations/registration_interface.hh"

namespace lh {
class NDTRegistration : public RegistrationInterface {
 public:
  NDTRegistration(const YAML::Node& node);
  NDTRegistration(float res, float step_size, float trans_eps, int max_iter);

  bool setInputTraget(const PointCloudPtr& input_target) override;
  bool scanMatch(const PointCloudPtr& input_source,
                 const Eigen::Matrix4f& predict_pose,
                 PointCloudPtr& result_cloud_ptr,
                 Eigen::Matrix4f result_pose) override;

 private:
  bool SetRegistrationParam(float res, float step_size, float trans_eps,
                            int max_iter);

 private:
  pcl::NormalDistributionsTransform<PointXYZ, PointXYZ>::Ptr ndt_ptr_;
};
}  // namespace lh
