#include "lidar_localization/models/graph_optimizer/interface_graph_optimizer.hpp"

namespace lidar_localization {
void InterfaceGraphOptimizer::SetMaxIterationsNum(int max_iterations_nums) {
  max_iterations_num_ = max_iterations_nums;
}
}  // namespace lidar_localization