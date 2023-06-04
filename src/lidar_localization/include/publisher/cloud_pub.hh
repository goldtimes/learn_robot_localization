#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include "sensor_data/cloud_data.hpp"
namespace lh {
class CloudPublisher {
 public:
  CloudPublisher() = default;
  CloudPublisher(ros::NodeHandle& nh, const std::string& topic, size_t buff_size,
                 const std::string& frame_id);
  void Publish(PointCloudPtr cloud_ptr);

 private:
  ros::NodeHandle nh_;
  ros::Publisher cloud_pub_;
  std::string frame_id_;
};
}  // namespace lh