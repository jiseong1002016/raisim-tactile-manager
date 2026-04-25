#pragma once

#include <Eigen/Dense>
#include <cstddef>
#include <vector>

namespace raisim_tactile {

struct RectangularGridConfig {
  int rows = 10;
  int cols = 10;
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  Eigen::Vector3d row_axis = Eigen::Vector3d::UnitY();
  Eigen::Vector3d col_axis = Eigen::Vector3d::UnitZ();
  double row_spacing = 0.0075;
  double col_spacing = 0.01352;
};

class SensorGrid {
 public:
  static SensorGrid rectangular(const RectangularGridConfig& config);
  static SensorGrid boltWrenchPad(bool left_side, double protrusion_m);

  const std::vector<Eigen::Vector3d>& cellsLocal() const { return cells_local_; }
  std::size_t size() const { return cells_local_.size(); }
  bool empty() const { return cells_local_.empty(); }

 private:
  std::vector<Eigen::Vector3d> cells_local_;
};

}  // namespace raisim_tactile
