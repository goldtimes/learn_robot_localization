#include "glog/logging.h"
#include "lidar_localization/lidar_undistortion/lidar_undstortion.hh"

namespace lidar_localization {
void LidarUndistortion::setMotionInfo(float scan_period,
                                      const VelocityData& velocity_data) {}

bool LidarUndistortion::undistortion(
    const CloudData::CLOUD_PTR& input_cloud_ptr,
    CloudData::CLOUD_PTR& out_cloud_pt) {}
Eigen::Matrix3f LidarUndistortion::updateMatrix(float real_time) {}
}  // namespace lidar_localization