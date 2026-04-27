#pragma once

#include <Eigen/Dense>
#include <vector>

#include "raisim_tactile/MeshSurface.hpp"
#include "raisim_tactile/SensorGrid.hpp"

namespace raisim_tactile {

struct HybridPenetrationTactileConfig {
  double mu = 0.4;
  double protrusion_m = 0.003;
  double vtan_threshold = 1.0e-4;
  bool use_contact_hint_sparse_cells = true;
  int max_cells_per_contact_hint = 25;
};

struct HybridTargetState {
  Eigen::Matrix3d orientation_W = Eigen::Matrix3d::Identity();
  Eigen::Vector3d position_W = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_velocity_W = Eigen::Vector3d::Zero();
};

struct HybridPadState {
  Eigen::Matrix3d orientation_W = Eigen::Matrix3d::Identity();
  Eigen::Vector3d position_W = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_velocity_W = Eigen::Vector3d::Zero();
  Eigen::Vector3d total_force_W = Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> contact_points_W;
};

struct HybridTactileResult {
  Eigen::VectorXd force_grid_flat = Eigen::VectorXd::Zero(600);
  std::vector<Eigen::Vector3d> left_contact_points_W;
  std::vector<Eigen::Vector3d> right_contact_points_W;
  std::size_t mesh_query_count = 0;
};

class HybridPenetrationTactile {
 public:
  HybridPenetrationTactile(HybridPenetrationTactileConfig config,
                           const MeshSurface* target_mesh,
                           const SensorGrid* left_grid,
                           const SensorGrid* right_grid);

  HybridTactileResult compute(const HybridTargetState& target,
                              const std::vector<HybridPadState>& left_pads,
                              const std::vector<HybridPadState>& right_pads) const;

 private:
  void accumulateSide(const HybridTargetState& target,
                      const std::vector<HybridPadState>& pads,
                      const SensorGrid& grid,
                      int flat_offset,
                      std::vector<Eigen::Vector3d>& contact_points_W,
                      Eigen::VectorXd& force_grid_flat,
                      std::size_t& mesh_query_count) const;

  std::vector<int> selectCandidateCells(const HybridPadState& pad,
                                        const SensorGrid& grid) const;

  HybridPenetrationTactileConfig config_;
  const MeshSurface* target_mesh_ = nullptr;
  const SensorGrid* left_grid_ = nullptr;
  const SensorGrid* right_grid_ = nullptr;
};

}  // namespace raisim_tactile
