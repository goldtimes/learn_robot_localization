#include "tf/tf_listener.hh"

#include <Eigen/Geometry>
namespace lh {
TFListener::TFListener(ros::NodeHandle& nh, const std::string& base_frame_id,
                       const std::string& child_frame_id)
    : nh_(nh), base_frame_id_(base_frame_id), child_frame_id_(child_frame_id) {}
bool TFListener::LookupData(Eigen::Matrix4f& transform_matrix) {
  try {
    tf::StampedTransform transform;
    listener_.lookupTransform(base_frame_id_, child_frame_id_, ros::Time(0),
                              transform);
    return TransformToMatrix(transform, transform_matrix);
  } catch (tf::TransformException& e) {
    ROS_ERROR("%s", e.what());
    return false;
  }
}

bool TFListener::TransformToMatrix(const tf::StampedTransform& transform,
                                   Eigen::Matrix4f& transform_matrix) {
  Eigen::Translation3f tl_btol(transform.getOrigin().getX(),
                               transform.getOrigin().getY(),
                               transform.getOrigin().getZ());

  double roll, pitch, yaw;
  tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
  Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());

  // 此矩阵为 child_frame_id 到 base_frame_id 的转换矩阵
  transform_matrix = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

  return true;
}

}  // namespace lh