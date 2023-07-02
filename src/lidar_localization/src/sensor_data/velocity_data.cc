#include "sensor_data/velocity_data.hpp"
#include "glog/logging.h"

namespace lh {
bool VelocityData::SyncData(std::deque<VelocityData>& unsynced_datas,
                            std::deque<VelocityData>& synced_datas,
                            double sync_time) {
  // sync_time 需要同步的雷达时间戳
  //   即找到与同步时间相邻的左右两个数据
  while (unsynced_datas.size() >= 2) {
    if (unsynced_datas.front().time > sync_time) {
      return false;
    }
    if (unsynced_datas.at(1).time < sync_time) {
      unsynced_datas.pop_front();
      continue;
    }
    // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
    if (sync_time - unsynced_datas.front().time > 0.2) {
      unsynced_datas.pop_front();
      break;
    }

    if (unsynced_datas.at(1).time - sync_time > 0.2) {
      unsynced_datas.pop_front();
      break;
    }
    break;
  }
  if (unsynced_datas.size() < 2) {
    return false;
  }

  VelocityData front_data = unsynced_datas.at(0);
  VelocityData back_data = unsynced_datas.at(1);
  VelocityData synced_data;
  // 线性插值
  double front_scale =
      (back_data.time - sync_time) / (back_data.time - front_data.time);
  double back_scale =
      (sync_time - front_data.time) / (back_data.time - front_data.time);
  synced_data.time = sync_time;
  synced_data.linear_velocity.x = front_data.linear_velocity.x * front_scale +
                                  back_data.linear_velocity.x * back_scale;
  synced_data.linear_velocity.y = front_data.linear_velocity.y * front_scale +
                                  back_data.linear_velocity.y * back_scale;
  synced_data.linear_velocity.z = front_data.linear_velocity.z * front_scale +
                                  back_data.linear_velocity.z * back_scale;
  synced_data.angular_velocity.x = front_data.angular_velocity.x * front_scale +
                                   back_data.angular_velocity.x * back_scale;
  synced_data.angular_velocity.y = front_data.angular_velocity.y * front_scale +
                                   back_data.angular_velocity.y * back_scale;
  synced_data.angular_velocity.z = front_data.angular_velocity.z * front_scale +
                                   back_data.angular_velocity.z * back_scale;

  synced_datas.push_back(synced_data);
}
}  // namespace lh