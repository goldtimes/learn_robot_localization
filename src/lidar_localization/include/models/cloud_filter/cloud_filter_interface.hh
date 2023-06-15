#include <yaml-cpp/yaml.h>
#include "sensor_data/cloud_data.hpp"

namespace lh {

class CloudFilterInterface {
 public:
  virtual ~CloudFilterInterface() = default;
  virtual bool Filter(const PointCloudPtr& input_cloud_ptr,
                      PointCloudPtr& filtered_cloud_ptr) = 0;
};
}  // namespace lh