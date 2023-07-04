#include "glog/logging.h"
#include "lidar_localization/lidar_undistortion/lidar_undstortion.hh"

namespace lidar_localization {
void LidarUndistortion::setMotionInfo(float scan_period,
                                      const VelocityData& velocity_data) {
  scan_period_ = scan_period;
  velocity_ << velocity_data.linear_velocity.x, velocity_data.linear_velocity.y,
      velocity_data.linear_velocity.z;
  angular_rate_ << velocity_data.angular_velocity.x,
      velocity_data.angular_velocity.y, velocity_data.angular_velocity.z;
}

bool LidarUndistortion::undistortion(
    const CloudData::CLOUD_PTR& input_cloud_ptr,
    CloudData::CLOUD_PTR& out_cloud_pt) {
  CloudData::CLOUD_PTR origin_cloud_ptr(new CloudData::CLOUD(*input_cloud_ptr));
  out_cloud_pt->points.clear();
  //   2*M_Pi
  float orientaition_space = 2.0 * M_PI;
  // 5度
  float delete_space = 5.0 * M_PI / 180.0;
  // 计算第一个点的角度
  float start_orientation =
      atan2(origin_cloud_ptr->points[0].y, origin_cloud_ptr->points[0].x);
  Eigen::AngleAxisf t_V(start_orientation, Eigen::Vector3d::UnitZ());
  Eigen::Matrix3f rotate_matrix = t_V.matrix();
  Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
  transform_matrix.block<3, 3>(0, 0) = rotate_matrix.inverse();
  pcl::transformPointCloud(*origin_cloud_ptr, *origin_cloud_ptr,
                           transform_matrix);
  // 这个速度是机器人坐标系的
  velocity_ = rotate_matrix * velocity_;
  angular_rate_ = rotate_matrix * angular_rate_;

  for (size_t point_index = 1; point_index < origin_cloud_ptr->size();
       ++point_index) {
    float orientation = atan2(origin_cloud_ptr->points[point_index].y,
                              origin_cloud_ptr->points[point_index].x);
    if (orientation < 0.0) orientation += 2.0 * M_PI;

    if (orientation < delete_space || 2.0 * M_PI - orientation < delete_space)
      continue;
    // 这是如何计算的？
    float real_time = fabs(orientation) / orientaition_space * scan_period_ -
                      scan_period_ / 2.0;
    Eigen::Vector3f origin_point(origin_cloud_ptr->points[point_index].x,
                                 origin_cloud_ptr->points[point_index].y,
                                 origin_cloud_ptr->points[point_index].z);
    Eigen::Matrix3f current_matrix = updateMatrix(real_time);
    Eigen::Vector3f rotation_point = current_matrix * origin_point;
    Eigen::Vector3f adjusted_point = rotation_point + velocity_ * real_time;

    CloudData::POINT point;
    point.x = adjusted_point(0);
    point.y = adjusted_point(1);
    point.z = adjusted_point(2);

    out_cloud_pt->points.push_back(point);
  }
  pcl::transformPointCloud(*out_cloud_pt, *out_cloud_pt,
                           transform_matrix.inverse());
  return true;
}
Eigen::Matrix3f LidarUndistortion::updateMatrix(float real_time) {
  Eigen::Vector3f angle = angular_rate_ * real_time;
  Eigen::AngleAxisf t_vz(angle(2), Eigen::Vector3f::UnitZ());
  Eigen::AngleAxisf t_vy(angle(1), Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf t_vx(angle(0), Eigen::Vector3f::UnitX());

  Eigen::AngleAxisf t_v;
  t_v = t_vz * t_vy * t_vx;
  return t_v.matrix();
}
}  // namespace lidar_localization