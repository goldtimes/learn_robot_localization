#pragma once

#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/stuff/macros.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

#include "lidar_localization/models/graph_optimizer/g2o/edge/edge_se3_priorquat.hpp"
#include "lidar_localization/models/graph_optimizer/g2o/edge/edge_se3_priorxyz.hpp"

#include "lidar_localization/models/graph_optimizer/interface_graph_optimizer.hpp"

namespace g2o {
class VertexSE3;
class VertexPlane;
class VertexPointXYZ;
class EdgeSE3;
class EdgeSE3Plane;
class EdgeSE3PointXYZ;
class EdgeSE3PriorXY;
class EdgeSE3PriorXYZ;
class EdgeSE3PriorVec;
class EdgeSE3PriorQuat;
class RobustKernelFactory;
}  // namespace g2o

G2O_USE_TYPE_GROUP(slam3d);
G2O_USE_OPTIMIZATION_LIBRARY(pcg);
G2O_USE_OPTIMIZATION_LIBRARY(cholmod);
G2O_USE_OPTIMIZATION_LIBRARY(csparse);

namespace lidar_localization {
class G2oGraphOptimizer : public InterfaceGraphOptimizer {
 public:
  G2oGraphOptimizer(const std::string& solver_type = "lm_var");
  // 优化
  bool Optimize() override;
  int GetNodeNum() override;
  // 输出数据
  bool GetOptimizedPose(std::deque<Eigen::Matrix4f>& optimized_pose) override;
  // 添加节点、边、鲁棒核
  void SetEdgeRobustKernel(std::string robust_kernel_name,
                           double robust_kernel_size) override;
  void AddSe3Node(const Eigen::Isometry3d& pose, bool need_fix) override;
  void AddSe3Edge(int vertex_index1, int vertex_index2,
                  const Eigen::Isometry3d& relative_pose,
                  const Eigen::VectorXd& noise) override;
  void AddSe3PriorXYZEdge(int se3_vertex_index, const Eigen::Vector3d& xyz,
                          const Eigen::VectorXd& noise) override;
  void AddSe3PriorQuaternionEdge(int se3_vertex_index,
                                 const Eigen::Quaterniond& quat,
                                 const Eigen::VectorXd& noise) override;

 private:
  Eigen::MatrixXd CalculateSe3EdgeInformationMatrix(
      const Eigen::VectorXd& noise);
  Eigen::MatrixXd CalculateSe3PriorQuaternionEdgeInformationMatrix(
      const Eigen::VectorXd& noise);
  Eigen::MatrixXd CalculateDiagMatrix(const Eigen::VectorXd& noise);
  void AddRobustKernel(g2o::OptimizableGraph::Edge* edge,
                       const std::string& kernel_type, double kernel_size);

 private:
  g2o::RobustKernelFactory* robust_kernel_factory_;
  std::unique_ptr<g2o::SparseOptimizer> graph_ptr_;

  std::string robust_kernel_name_;
  double robust_kernel_size_;
  bool need_robust_kernel_ = false;
};
}  // namespace lidar_localization