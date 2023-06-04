#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_GNSS_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_GNSS_SUBSCRIBER_HPP_

#include <ros/ros.h>
#include <deque>
#include "sensor_data/gnss_data.hpp"
#include "sensor_msgs/NavSatFix.h"

namespace lh {
class GNSSSubscriber {
 public:
  GNSSSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
  GNSSSubscriber() = default;
  void ParseData(std::deque<GNSSData>& deque_gnss_data);

 private:
  void msg_callback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  // 为什么每个sub 要定义queue来存储数据呢？
  // 比如gnss的数据是100hz,而lidar的；频率为10hz。
  // 并且我们是在lidar中去做数据融合的，经过融合的算法之后
  // gnss的数据要比lidar提前很多，那么下一次在融合的时候，数据的时间就不是一样的了。
  std::deque<GNSSData> new_gnss_data_;
};
}  // namespace lh
#endif