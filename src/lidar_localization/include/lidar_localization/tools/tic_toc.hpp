/**
 * 测试运行时间
 */

#pragma once

#include <chrono>
#include <cstdlib>
#include <ctime>

namespace lidar_localization {
class TicToc {
 public:
  TicToc() { tic(); }
  void tic() { start = std::chrono::system_clock::now(); }
  double toc() {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> time_used = end - start;
    start = std::chrono::system_clock::now();
    return time_used.count();
  }

 private:
  std::chrono::time_point<std::chrono::system_clock> start;
  std::chrono::time_point<std::chrono::system_clock> end;
};
}  // namespace lidar_localization