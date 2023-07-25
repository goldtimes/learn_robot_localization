#pragma once

#include <Eigen/Dense>
#include <deque>
#include <string>

namespace lidar_localization {
class InterfaceGraphOptimizer {
 public:
  virtual ~InterfaceGraphOptimizer() {}
  //优化
  virtual bool Optimize() = 0;
  // 输入输出数据
  virtual bool GetOptimizedPose(
      std::deque<Eigen::Matrix4f>& optimized_pose) = 0;
  virtual int GetNodeNum() = 0;
  // 添加节点、边、鲁棒核
  virtual void SetEdgeRobustKernel(std::string robust_kernel_name,
                                   double robust_kernel_size) = 0;
  virtual void AddSe3Node(const Eigen::Isometry3d& pose, bool need_fix) = 0;
  virtual void AddSe3Edge(int vertex_index1, int vertex_index2,
                          const Eigen::Isometry3d& relative_pose,
                          const Eigen::VectorXd& noise) = 0;
  virtual void AddSe3PriorXYZEdge(int se3_vertex_index,
                                  const Eigen::Vector3d& xyz,
                                  const Eigen::VectorXd& noise) = 0;
  virtual void AddSe3PriorQuaternionEdge(int se3_vertex_index,
                                         const Eigen::Quaterniond& quat,
                                         const Eigen::VectorXd& noise) = 0;
  // 设置优化参数
  void SetMaxIterationsNum(int max_iterations_num);

 private:
 protected:
  int max_iterations_num_ = 512;
};
}  // namespace lidar_localization