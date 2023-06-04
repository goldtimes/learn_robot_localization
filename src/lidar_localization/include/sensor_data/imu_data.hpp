#pragma once

#include <Eigen/Dense>

namespace lh {
class ImuData {
 public:
  struct LinearAccelation {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };

  struct AngularVelocity {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };

  struct Orientation {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;
  };

  double time = 0.0;
  LinearAccelation linear_acc_;
  AngularVelocity angular_vel_;
  Orientation orientation_;

 public:
  Eigen::Matrix3f getOrientationMatrix() {
    // clang-format off
    Eigen::Quaterniond q(orientation_.w, orientation_.x, 
        orientation_.y, orientation_.z);
    //
    // q.toRotationMatrix();
    Eigen::Matrix3f matrix = q.matrix().cast<float>();
    return matrix;
  }
};
}  // namespace lidar_localization