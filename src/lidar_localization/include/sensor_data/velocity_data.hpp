#pragma once

#include <Eigen/Dense>
#include <deque>

namespace lh {

struct LinearVelocity {
  /* data */
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct AngularVelocity {
  /* data */
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

class VelocityData {
 public:
  double time = 0.0;
  LinearVelocity linear_velocity;
  AngularVelocity angular_velocity;

  static bool SyncData(std::deque<VelocityData>& unsync_datas,
                       std::deque<VelocityData>& sync_datas, double sync_time);
};
}  // namespace lh