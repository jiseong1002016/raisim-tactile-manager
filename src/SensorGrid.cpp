#include "raisim_tactile/SensorGrid.hpp"

namespace raisim_tactile {

SensorGrid SensorGrid::rectangular(const RectangularGridConfig& config) {
  SensorGrid grid;
  if (config.rows <= 0 || config.cols <= 0) return grid;

  Eigen::Vector3d row_axis = config.row_axis;
  Eigen::Vector3d col_axis = config.col_axis;
  if (row_axis.squaredNorm() > 1.0e-24) row_axis.normalize();
  if (col_axis.squaredNorm() > 1.0e-24) col_axis.normalize();

  grid.cells_local_.reserve(static_cast<std::size_t>(config.rows * config.cols));
  for (int row = 0; row < config.rows; ++row) {
    for (int col = 0; col < config.cols; ++col) {
      grid.cells_local_.push_back(
          config.origin +
          row * config.row_spacing * row_axis +
          col * config.col_spacing * col_axis);
    }
  }
  return grid;
}

SensorGrid SensorGrid::boltWrenchPad(bool left_side, double protrusion_m) {
  const double x_sign = left_side ? 1.0 : -1.0;
  RectangularGridConfig config;
  config.rows = 10;
  config.cols = 10;
  config.origin = Eigen::Vector3d(x_sign * (0.01091 + protrusion_m), 0.003750, -0.060840);
  config.row_axis = Eigen::Vector3d::UnitY();
  config.col_axis = Eigen::Vector3d::UnitZ();
  config.row_spacing = 0.007500;
  config.col_spacing = 0.013520;
  return rectangular(config);
}

}  // namespace raisim_tactile
