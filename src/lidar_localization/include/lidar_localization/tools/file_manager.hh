#pragma once

#include <fstream>
#include <iostream>
#include <string>

namespace lidar_localization {
class FileManager {
 public:
  static bool CreateFile(std::ofstream& ofs, std::string file_path);
  static bool CreateDirectory(std::string directory_path);
};
}  // namespace lidar_localization