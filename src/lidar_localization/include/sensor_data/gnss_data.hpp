#pragma once

#include <string>
#include <vector>

#include <Geocentric/LocalCartesian.hpp>
#include <deque>

namespace lh {
class GNSSData {
 public:
  double time = 0.0;
  double longtitude = 0.0;
  double latitude = 0.0;
  double altitude = 0.0;

  double local_E = 0.0;
  double local_N = 0.0;
  double local_U = 0.0;
  int status = 0;
  int service = 0;

 private:
  static GeographicLib::LocalCartesian geo_converter;
  static bool origin_position_inited;

 public:
  void InitOriginPosition();
  void UpdateXYZ();
  static bool SyncData(std::deque<GNSSData>& UnsyncedData,
                       std::deque<GNSSData>& SyncedData, double sync_time);
};
}  // namespace lh