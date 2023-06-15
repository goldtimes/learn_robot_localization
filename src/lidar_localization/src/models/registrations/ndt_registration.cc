#include "models/registrations/ndt_registration.hh"
#include "glog/logging.h"

namespace lh {
NDTRegistration::NDTRegistration(const YAML::node& node) {}

NDTRegistration::NDTRegistration(float res, float step_size, float trans_eps,
                                 int max_iter) {}

bool NDTRegistration::SetRegistrationParam(float res, float step_size,
                                           float trans_eps, int max_iter) {}

bool NDTRegistration::setInputTraget(const PointCloudPtr& input_target) {}
bool NDTRegistration::scanMatch(const PointCloudPtr& input_source,
                                const Eigen::Matrix4f& predict_pose,
                                PointCloudPtr& result_cloud_ptr,
                                Eigen::Matrix4f result_pose) {}
}  // namespace lh
