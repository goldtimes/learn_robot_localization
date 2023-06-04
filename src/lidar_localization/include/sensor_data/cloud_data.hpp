#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <boost/shared_ptr.hpp>

namespace lh {
using PointXYZ = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointXYZ>;
using PointCloudPtr = PointCloud::Ptr;

class CloudData {
 public:
  CloudData() : cloud_ptr_(boost::make_shared<PointCloud>()) {}

 public:
  double time = 0.0;
  PointCloudPtr cloud_ptr_;
  // 居然不能使用boost make_shared
};
}  // namespace lh
