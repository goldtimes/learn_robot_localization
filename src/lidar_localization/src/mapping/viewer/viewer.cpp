#include "lidar_localization/mapping/viewer/viewer.hh"

#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/tools/file_manager.hh"

namespace lidar_localization {
Viewer::Viewer() { InitWithConfig(); }
}  // namespace lidar_localization