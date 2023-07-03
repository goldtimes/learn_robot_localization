#include "lidar_localization/tools/file_manager.hh"

#include <boost/filesystem.hpp>
#include "glog/logging.h"

namespace lidar_localization {
bool FileManager::CreateFile(std::ofstream& ofs, std::string file_path) {
  ofs.open(file_path.c_str(), std::ios::app);
  if (!ofs) {
    LOG(WARNING) << "无法生成文件: " << file_path;
    return false;
  }
  return true;
}

bool FileManager::CreateDirectory(std::string directory_path) {
  if (!boost::filesystem::is_directory(directory_path)) {
    boost::filesystem::create_directories(directory_path);
  }
  if (!boost::filesystem::is_directory(directory_path)) {
    LOG(WARNING) << "无法生成目录: " << directory_path;
  }
  return true;
}
}  // namespace lidar_localization