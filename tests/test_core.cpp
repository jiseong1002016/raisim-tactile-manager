#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>

#include "raisim_tactile/HybridPenetrationTactile.hpp"
#include "raisim_tactile/MeshSurface.hpp"
#include "raisim_tactile/SensorGrid.hpp"

namespace {

void writeTriangleObj(const std::string& path) {
  std::ofstream out(path);
  out << "v -0.1 -0.1 0.0\n";
  out << "v 0.1 -0.1 0.0\n";
  out << "v 0.0 0.1 0.0\n";
  out << "vn 0.0 0.0 1.0\n";
  out << "f 1//1 2//1 3//1\n";
}

}  // namespace

int main() {
  const std::string mesh_path = "test_triangle.obj";
  writeTriangleObj(mesh_path);

  raisim_tactile::MeshSurface mesh;
  assert(mesh.loadObj(mesh_path, 1.0));
  assert(mesh.vertexCount() == 3);
  assert(mesh.triangleCount() == 1);

  const auto query = mesh.closestPoint(Eigen::Vector3d(0.0, 0.0, 0.002));
  assert(query.triangle_index == 0);
  assert(std::abs(query.abs_distance - 0.002) < 1.0e-12);

  const auto left_grid = raisim_tactile::SensorGrid::boltWrenchPad(true, 0.003);
  const auto right_grid = raisim_tactile::SensorGrid::boltWrenchPad(false, 0.003);
  assert(left_grid.size() == 100);
  assert(right_grid.size() == 100);

  raisim_tactile::HybridPenetrationTactileConfig cfg;
  cfg.protrusion_m = 0.010;
  raisim_tactile::HybridPenetrationTactile tactile(cfg, &mesh, &left_grid, &right_grid);

  raisim_tactile::HybridTargetState target;
  raisim_tactile::HybridPadState pad;
  pad.position_W = Eigen::Vector3d(0.0, -0.04, 0.0);
  pad.total_force_W = Eigen::Vector3d(0.0, 0.0, 10.0);

  const auto result = tactile.compute(target, {pad}, {});
  assert(result.force_grid_flat.size() == 600);
  assert(result.force_grid_flat.allFinite());
  assert(result.force_grid_flat.norm() > 0.0);
  assert(result.left_contact_points_W.size() == 100);
  assert(result.right_contact_points_W.size() == 100);

  std::cout << "raisim_tactile_core_test OK\n";
  return 0;
}
