#include "raisim_tactile/HybridPenetrationTactile.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace raisim_tactile {

HybridPenetrationTactile::HybridPenetrationTactile(
    HybridPenetrationTactileConfig config,
    const MeshSurface* target_mesh,
    const SensorGrid* left_grid,
    const SensorGrid* right_grid)
    : config_(config),
      target_mesh_(target_mesh),
      left_grid_(left_grid),
      right_grid_(right_grid) {
}

HybridTactileResult HybridPenetrationTactile::compute(
    const HybridTargetState& target,
    const std::vector<HybridPadState>& left_pads,
    const std::vector<HybridPadState>& right_pads) const {
  HybridTactileResult result;
  result.left_contact_points_W.assign(100, Eigen::Vector3d::Constant(
                                               std::numeric_limits<double>::quiet_NaN()));
  result.right_contact_points_W.assign(100, Eigen::Vector3d::Constant(
                                                std::numeric_limits<double>::quiet_NaN()));

  if (target_mesh_ == nullptr || target_mesh_->empty()) return result;
  if (left_grid_ != nullptr) {
    accumulateSide(target, left_pads, *left_grid_, 0,
                   result.left_contact_points_W, result.force_grid_flat);
  }
  if (right_grid_ != nullptr) {
    accumulateSide(target, right_pads, *right_grid_, 300,
                   result.right_contact_points_W, result.force_grid_flat);
  }
  return result;
}

void HybridPenetrationTactile::accumulateSide(
    const HybridTargetState& target,
    const std::vector<HybridPadState>& pads,
    const SensorGrid& grid,
    int flat_offset,
    std::vector<Eigen::Vector3d>& contact_points_W,
    Eigen::VectorXd& force_grid_flat) const {
  const auto& cells_local = grid.cellsLocal();
  if (cells_local.size() < 100) return;

  const Eigen::Matrix3d R_w_inv = target.orientation_W.transpose();
  std::array<double, 100> point_weight{};
  std::array<Eigen::Vector3d, 100> point_sum{};
  for (auto& p : point_sum) p.setZero();

  for (const auto& pad : pads) {
    std::array<double, 100> pen{};
    std::array<Eigen::Vector3d, 100> surface_normal{};
    std::fill(pen.begin(), pen.end(), 0.0);
    for (auto& n : surface_normal) n.setZero();
    double sum_pen = 0.0;

    for (int i = 0; i < 100; ++i) {
      const Eigen::Vector3d cell_W = pad.position_W + pad.orientation_W * cells_local[i];
      const Eigen::Vector3d cell_target = R_w_inv * (cell_W - target.position_W);
      const auto surface_query = target_mesh_->closestPoint(cell_target);
      const double depth = config_.protrusion_m - surface_query.abs_distance;
      pen[i] = std::max(0.0, depth);
      surface_normal[i] = surface_query.normal;
      sum_pen += pen[i];
    }

    if (sum_pen < 1.0e-12 || pad.total_force_W.norm() < 1.0e-9) continue;

    const double total_force_mag = pad.total_force_W.norm();
    const Eigen::Vector3d v_rel_W = pad.linear_velocity_W - target.linear_velocity_W;
    for (int i = 0; i < 100; ++i) {
      if (pen[i] <= 0.0 || surface_normal[i].squaredNorm() < 1.0e-24) continue;
      const double weight = pen[i] / sum_pen;
      const double cell_force_mag = total_force_mag * weight;
      Eigen::Vector3d n_W = target.orientation_W * surface_normal[i];
      const Eigen::Vector3d cell_W = pad.position_W + pad.orientation_W * cells_local[i];
      if (n_W.dot(cell_W - target.position_W) < 0.0) n_W = -n_W;

      const Eigen::Vector3d F_normal = cell_force_mag * n_W;
      const Eigen::Vector3d v_tan = v_rel_W - v_rel_W.dot(n_W) * n_W;
      Eigen::Vector3d F_shear = Eigen::Vector3d::Zero();
      if (v_tan.norm() > config_.vtan_threshold) {
        F_shear = -config_.mu * cell_force_mag * v_tan.normalized();
      }

      const Eigen::Vector3d F_cell = F_normal + F_shear;
      force_grid_flat[flat_offset + i * 3 + 0] += F_cell[0];
      force_grid_flat[flat_offset + i * 3 + 1] += F_cell[1];
      force_grid_flat[flat_offset + i * 3 + 2] += F_cell[2];

      const double contact_weight = F_cell.norm();
      if (std::isfinite(contact_weight) && contact_weight > 1.0e-12) {
        point_sum[i] += contact_weight * cell_W;
        point_weight[i] += contact_weight;
      }
    }
  }

  for (int i = 0; i < 100; ++i) {
    if (point_weight[i] > 1.0e-12) {
      contact_points_W[i] = point_sum[i] / point_weight[i];
    }
  }
}

}  // namespace raisim_tactile
