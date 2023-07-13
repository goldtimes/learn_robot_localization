#include "lidar_localization/mapping/viewer/viewer.hh"

#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/tools/file_manager.hh"

namespace lidar_localization {
Viewer::Viewer() { InitWithConfig(); }

bool Viewer::InitWithConfig() {
  std::string config_file_path = WORK_SPACE_PATH + "/config/viewer/config.yaml";
  YAML::Node config_node = YAML::LoadFile(config_file_path);

  InitParam(config_node);
  InitDataPath(config_node);
  InitFilter("frame", frame_filter_ptr_, config_node);
  InitFilter("local_map", local_map_filter_ptr_, config_node);
  InitFilter("global_map", global_map_filter_ptr_, config_node);

  return true;
}

bool Viewer::InitParam(const YAML::Node& config_node) {
  local_frame_num_ = config_node["local_frame_num"].as<int>();
  return true;
}

bool Viewer::InitDataPath(const YAML::Node& config_node) {
  std::string data_path = config_node["data_path"].as<std::string>();
  if (data_path == "./") {
    data_path = WORK_SPACE_PATH;
  }

  key_frames_path_ = data_path + "/slam_data/key_frames";
  map_path_ = data_path + "/slam_data/map";

  // if (!FileManager::InitDirectory(map_path_, "点云地图文件")) return false;

  return true;
}

bool Viewer::InitFilter(const std::string& filter_user,
                        std::shared_ptr<CloudFilterInterface>& filter_ptr,
                        const YAML::Node& config_node) {
  std::string filter_mothod =
      config_node[filter_user + "_filter"].as<std::string>();
  LOG(INFO) << "viewer_" + filter_user << "选择的滤波方法为：" << filter_mothod;

  if (filter_mothod == "voxel_filter") {
    filter_ptr =
        std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);
  } else {
    LOG(ERROR) << "没有为 " << filter_user << " 找到与 " << filter_mothod
               << " 相对应的滤波方法!";
    return false;
  }

  return true;
}

bool Viewer::HasNewLocalMap() { return has_new_local_map_; }

bool Viewer::HasNewGlobalMap() { return has_new_global_map_; }

}  // namespace lidar_localization