# learn_robot_localization
知乎中的任乾的自动驾驶定位学习

## Content
1. 3D激光slam，地图的建立与基于点云地图的定位
2. 仅仅依赖于激光雷达定位，容易受环境的影响，雷达被屏蔽或者空旷的地带，
需要结合IMU、GNSS、轮速计等信息
3. 组合导航
    1. IMU + GNSS组合导航。
    2. 点云地图定位+IMU+GNSS的融合
4. 激光雷达的前端会使用ndt、icp、特征匹配
5. 滤波算法: EKF, 粒子滤波
6. 后端优化: g2o, gtsam, ceres.

## 数据集 
[https://pan.baidu.com/share/init?surl=TyXbifoTHubu3zt4jZ90Wg](n9ys)



## Third_party
eigen和geographicLib
可以用过apt安装

## 前端里程计 front_end模块
点云匹配方法，通过匹配来累积位姿。我们这里选择NDT匹配方法。
### 算法的设计思路
1. 接收到第一帧的点云开始，那么第一帧就是地图，供下一帧的数据来匹配 
2. 当时如果每帧的数据都加入地图，那么地图的数据太大了。所以我们要提取关键帧，每隔一段距离来取一帧点云。
  利用关键帧拼接地图
3. 但是可以想象，即使用关键帧的方式，那么拼接的地图也很大，比如走过了100m。用当前帧去和之前100m的地图匹配，那么也很多无效的匹配，所以我们需要用一个滑动窗口来保存地图，利用当前帧和滑动窗口中的地图去匹配。
4. 匹配之前进行滤波，对点云稀疏化，加速匹配的速度。
5. 匹配算法对于初始的位姿非常敏感，不能以上一帧的姿态作为这一帧匹配的预测值

### 点云匹配
pcl的ndt匹配算法

## 为了降低功能的耦合，需要重构优化代码
前端里程计包含了滤波，局部地图滑动窗口，匹配，全局地图等。
1. 匹配的功能模块呢，后端闭环检测也是需要的，地图定位也是需要的。
2. 滤波算法也是许多模块需要的，滤波的参数也不能如现在一样，给固定值
3. 匹配方式的升级，现在用ndt,那么如果要支持icp呢？多态继承的方式


## 里程计精度的评估
### 采用开源的评价方法evo
1. 录制gnss数据和里程计数据
2. 将数据交给evo处理

### 安装 evo
```bash
pip install evo --upgrade --no-binary evo
```

### 评价
#### evo_rpe 每段距离内的误差评价
```bash
# kitti的odometry榜单中的距离误差指标
evo_rpe kitti ground_truth.txt laser_odom.txt -r trans_part --delta 100 --plot --plot_mode xyz
```

#### evo_ape 绝对误差随路程的评价
```bash
# kitti的odometry榜单中的距离误差指标
evo_ape kitti ground_truth.txt laser_odom.txt -r full --plot --plot_mode xyz
```
