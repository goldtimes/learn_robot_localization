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

