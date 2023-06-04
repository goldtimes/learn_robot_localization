#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <string>

namespace lh {
class TFListener {
 public:
  TFListener(ros::NodeHandle& nh, const std::string& base_frame_id,
             const std::string& child_frame_id);
  TFListener() = default;
  bool LookupData(Eigen::Matrix4f& transform_matrix);

 private:
  bool TransformToMatrix(const tf::StampedTransform& transform,
                         Eigen::Matrix4f& matrix);

 private:
  ros::NodeHandle nh_;
  tf::TransformListener listener_;
  std::string base_frame_id_;
  std::string child_frame_id_;
};
}  // namespace lh
