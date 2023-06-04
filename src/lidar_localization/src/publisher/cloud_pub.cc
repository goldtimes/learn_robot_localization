#include "publisher/cloud_pub.hh"

namespace lh {
CloudPublisher::CloudPublisher(ros::NodeHandle& nh, const std::string& topic_name, size_t buff_size,
                               const std::string& frame_id)
    : nh_(nh), frame_id_(frame_id) {
  cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, buff_size);
}

void CloudPublisher::Publish(PointCloudPtr cloud_ptr) {
  sensor_msgs::PointCloud2::Ptr cloud_out(new sensor_msgs::PointCloud2());
  pcl::toROSMsg(*cloud_ptr, *cloud_out);
  cloud_out->header.stamp = ros::Time::now();
  cloud_out->header.frame_id = frame_id_;
  cloud_pub_.publish(cloud_out);
}

}  // namespace lh