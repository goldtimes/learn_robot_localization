#pragma once

#include <string>
#include <vector>

#include <Geocentric/LocalCartesian.hpp>

namespace lidar_localization {
class GNNSData {
 public:
  double time = 0.0;
  double longtitude = 0.0;
  double latitude = 0.0;
  double loccal_E = 0.0;
  double local_N = 0.0;
  double local_U = 0.0;
  int status = 0;
  int service = 0;

 private:
  static bool origin_position_inited;

 public:
  void InitOriginPosition();
  void UpdateXYZ();
};
}  // namespace lidar_localization