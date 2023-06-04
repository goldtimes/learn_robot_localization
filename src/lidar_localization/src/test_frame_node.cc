#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <ros/ros.h>

#include "global_defination/global_defination.h"
#include "publisher/cloud_pub.hh"
#include "publisher/odom_pub.hh"
#include "subscriber/cloud_sub.hh"
#include "subscriber/gnss_sub.hh"
#include "subscriber/imu_sub.hh"
#include "tf/tf_listener.hh"

using namespace lh;

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_log_dir = WORK_SPACE_PATH + "/log";
  FLAGS_alsologtostderr = 1;

  ros::init(argc, argv, "test_frame_node");
  ros::NodeHandle nh;
  // 监听点云数据
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr =
      std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);
  // 监听imu数据
  std::shared_ptr<IMUSub> imu_sub_ptr =
      std::make_shared<IMUSub>(nh, "/kitti/oxts/imu", 1000000);
  // 监听gnss数据
  std::shared_ptr<GNSSSubscriber> gnss_sub_ptr =
      std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);
  // 获得激光->imu的坐标变换
  std::shared_ptr<TFListener> lidar_to_imu_ptr =
      std::make_shared<TFListener>(nh, "velo_link", "imu_link");
  // 发布当前的点云
  std::shared_ptr<CloudPublisher> cloud_pub_ptr =
      std::make_shared<CloudPublisher>(nh, "current_scan", 100, "/map");
  // 里程计的发布 lidar odom
  std::shared_ptr<OdomPublisher> odom_pub_ptr =
      std::make_shared<OdomPublisher>(nh, "lidar_odom", "map", "lidar", 100);
  std::deque<CloudData> cloud_data_buff;
  std::deque<ImuData> imu_data_buff;
  std::deque<GNSSData> gnss_data_buff;
  Eigen::Matrix4f lidar_to_imu = Eigen::Matrix4f::Identity();

  bool transform_received = false;
  bool gnss_origin_position_inited = false;

  ros::Rate rate(100);
  while (ros::ok()) {
    // 先回调一次，读取传感器的数据
    ros::spinOnce();
    // 将之前拿到的数据放到队列中
    cloud_sub_ptr->ParseData(cloud_data_buff);
    imu_sub_ptr->ParseData(imu_data_buff);
    gnss_sub_ptr->ParseData(gnss_data_buff);

    if (!transform_received) {
      // 获得lidar->imu的坐标变换
      if (lidar_to_imu_ptr->LookupData(lidar_to_imu)) {
        transform_received = true;
      }
    } else {
      while (cloud_data_buff.size() > 0 && imu_data_buff.size() > 0 &&
             gnss_data_buff.size() > 0) {
        CloudData earliest_cloud_data = cloud_data_buff.front();
        ImuData earliest_imu_data = imu_data_buff.front();
        GNSSData earliest_gnss_data = gnss_data_buff.front();
        //
        double dt = earliest_cloud_data.time - earliest_imu_data.time;
        if (dt < -0.05) {
          cloud_data_buff.pop_front();
        } else if (dt > 0.05) {
          imu_data_buff.pop_front();
          gnss_data_buff.pop_front();
        } else {
          cloud_data_buff.pop_front();
          imu_data_buff.pop_front();
          gnss_data_buff.pop_front();

          Eigen::Matrix4f odometry_matrix;

          if (!gnss_origin_position_inited) {
            earliest_gnss_data.InitOriginPosition();
            gnss_origin_position_inited = true;
          }
          earliest_gnss_data.UpdateXYZ();
          odometry_matrix(0, 3) = earliest_gnss_data.local_E;
          odometry_matrix(1, 3) = earliest_gnss_data.local_N;
          odometry_matrix(2, 3) = earliest_gnss_data.local_U;
          odometry_matrix.block<3, 3>(0, 0) =
              earliest_imu_data.getOrientationMatrix();
          odometry_matrix = odometry_matrix * lidar_to_imu;
          pcl::transformPointCloud(*earliest_cloud_data.cloud_ptr_,
                                   *earliest_cloud_data.cloud_ptr_,
                                   odometry_matrix);
          cloud_pub_ptr->Publish(earliest_cloud_data.cloud_ptr_);
          odom_pub_ptr->Publish(odometry_matrix);
        }
      }
    }
    rate.sleep();
  }

  return 0;
}