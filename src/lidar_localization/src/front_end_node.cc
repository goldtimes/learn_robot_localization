#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <ros/ros.h>

#include <memory>
#include "front_end/front_end_flow.hh"
#include "global_defination/global_defination.h"
#include "publisher/cloud_pub.hh"
#include "publisher/odom_pub.hh"
#include "subscriber/cloud_sub.hh"
#include "subscriber/gnss_sub.hh"
#include "subscriber/imu_sub.hh"
#include "subscriber/velocity_sub.hh"
#include "tf/tf_listener.hh"

using namespace lh;

std::shared_ptr<FrontEndFlow> _front_end_flow_ptr;

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
  FLAGS_alsologtostderr = 1;

  ros::init(argc, argv, "front_end_node");
  ros::NodeHandle nh;

  // ros::ServiceServer service =
  //     nh.advertiseService("save_map", save_map_callback);
  _front_end_flow_ptr = std::make_shared<FrontEndFlow>(nh);

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();

    _front_end_flow_ptr->Run();

    rate.sleep();
  }

  return 0;
}

// int main(int argc, char *argv[]) {
//   google::InitGoogleLogging(argv[0]);
//   google::InstallFailureSignalHandler();

//   FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
//   FLAGS_alsologtostderr = 1;

//   ros::init(argc, argv, "front_end_node");
//   ros::NodeHandle nh;

//   std::shared_ptr<CloudSubscriber> cloud_sub_ptr =
//       std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud",
//       100000);
//   std::shared_ptr<VelocitySubscriber> velocity_sub_ptr_ =
//       std::make_shared<VelocitySubscriber>(nh, "/kitti/oxts/gps/vel",
//       1000000);

//   std::shared_ptr<IMUSub> imu_sub_ptr =
//       std::make_shared<IMUSub>(nh, "/kitti/oxts/imu", 1000000);
//   std::shared_ptr<GNSSSubscriber> gnss_sub_ptr =
//       std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);
//   std::shared_ptr<TFListener> lidar_to_imu_ptr =
//       std::make_shared<TFListener>(nh, "velo_link", "imu_link");

//   std::shared_ptr<CloudPublisher> cloud_pub_ptr =
//       std::make_shared<CloudPublisher>(nh, "current_scan", 100, "/map");
//   std::shared_ptr<CloudPublisher> local_map_pub_ptr =
//       std::make_shared<CloudPublisher>(nh, "local_map", 100, "/map");
//   std::shared_ptr<CloudPublisher> global_map_pub_ptr =
//       std::make_shared<CloudPublisher>(nh, "global_map", 100, "/map");
//   std::shared_ptr<OdomPublisher> laser_odom_pub_ptr =
//       std::make_shared<OdomPublisher>(nh, "laser_odom", "map", "lidar", 100);
//   std::shared_ptr<OdomPublisher> gnss_pub_ptr =
//       std::make_shared<OdomPublisher>(nh, "gnss", "map", "lidar", 100);

//   std::shared_ptr<FrontEnd> front_end_ptr = std::make_shared<FrontEnd>();

//   std::deque<CloudData> cloud_data_buff;
//   std::deque<ImuData> imu_data_buff;
//   std::deque<GNSSData> gnss_data_buff;
//   std::deque<VelocityData> velocity_data_buff;
//   Eigen::Matrix4f lidar_to_imu = Eigen::Matrix4f::Identity();
//   bool transform_received = false;
//   bool gnss_origin_position_inited = false;
//   bool front_end_pose_inited = false;
//   PointCloudPtr local_map_ptr(new PointCloud());
//   PointCloudPtr global_map_ptr(new PointCloud());
//   PointCloudPtr current_scan_ptr(new PointCloud());

//   double run_time = 0.0;
//   double init_time = 0.0;
//   bool time_inited = false;
//   bool has_global_map_published = false;

//   static bool sensor_inited = false;

//   ros::Rate rate(100);
//   while (ros::ok()) {
//     ros::spinOnce();

//     cloud_sub_ptr->ParseData(cloud_data_buff);

//     static std::deque<ImuData> unsynced_imu_;
//     static std::deque<VelocityData> unsynced_velocity_;
//     static std::deque<GNSSData> unsynced_gnss_;

//     imu_sub_ptr->ParseData(unsynced_imu_);
//     gnss_sub_ptr->ParseData(unsynced_gnss_);

//     double cloud_time = cloud_data_buff.front().time;
//     bool valid_imu =
//         ImuData::SyncData(unsynced_imu_, imu_data_buff, cloud_time);
//     bool valid_velocity = VelocityData::SyncData(
//         unsynced_velocity_, velocity_data_buff, cloud_time);
//     bool valid_gnss =
//         GNSSData::SyncData(unsynced_gnss_, gnss_data_buff, cloud_time);

//     if (!sensor_inited) {
//       if (!valid_imu || !valid_velocity || !valid_gnss) {
//         if (!cloud_data_buff.empty()) cloud_data_buff.pop_front();
//       } else {
//         sensor_inited = true;
//       }
//     }

//     if (sensor_inited) {
//       if (!transform_received) {
//         if (lidar_to_imu_ptr->LookupData(lidar_to_imu)) {
//           transform_received = true;
//         }
//       } else {
//         while (cloud_data_buff.size() > 0 && imu_data_buff.size() > 0 &&
//                gnss_data_buff.size() > 0) {
//           CloudData cloud_data = cloud_data_buff.front();
//           ImuData imu_data = imu_data_buff.front();
//           GNSSData gnss_data = gnss_data_buff.front();

//           if (!time_inited) {
//             time_inited = true;
//             init_time = cloud_data.time;
//           } else {
//             run_time = cloud_data.time - init_time;
//           }

//           double d_time = cloud_data.time - imu_data.time;
//           if (d_time < -0.05) {
//             cloud_data_buff.pop_front();
//           } else if (d_time > 0.05) {
//             imu_data_buff.pop_front();
//             gnss_data_buff.pop_front();
//           } else {
//             cloud_data_buff.pop_front();
//             imu_data_buff.pop_front();
//             gnss_data_buff.pop_front();

//             Eigen::Matrix4f odometry_matrix = Eigen::Matrix4f::Identity();

//             if (!gnss_origin_position_inited) {
//               gnss_data.InitOriginPosition();
//               gnss_origin_position_inited = true;
//             }
//             gnss_data.UpdateXYZ();
//             odometry_matrix(0, 3) = gnss_data.local_E;
//             odometry_matrix(1, 3) = gnss_data.local_N;
//             odometry_matrix(2, 3) = gnss_data.local_U;
//             odometry_matrix.block<3, 3>(0, 0) =
//             imu_data.getOrientationMatrix(); odometry_matrix *= lidar_to_imu;
//             gnss_pub_ptr->Publish(odometry_matrix);

//             if (!front_end_pose_inited) {
//               front_end_pose_inited = true;
//               front_end_ptr->SetInitPose(odometry_matrix);
//             }
//             front_end_ptr->SetPredictPose(odometry_matrix);
//             Eigen::Matrix4f laser_matrix = front_end_ptr->Update(cloud_data);
//             laser_odom_pub_ptr->Publish(laser_matrix);

//             front_end_ptr->GetCurrentScan(current_scan_ptr);
//             cloud_pub_ptr->Publish(current_scan_ptr);
//             if (front_end_ptr->GetNewLocalMap(local_map_ptr))
//               local_map_pub_ptr->Publish(local_map_ptr);
//           }
//           if (run_time > 460.0 && !has_global_map_published) {
//             if (front_end_ptr->GetNewGlobalMap(global_map_ptr)) {
//               global_map_pub_ptr->Publish(global_map_ptr);
//               has_global_map_published = true;
//             }
//           }
//         }
//       }
//     }

//     rate.sleep();
//   }

//   return 0;
// }