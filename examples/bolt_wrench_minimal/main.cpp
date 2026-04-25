#include <iostream>
#include <string>

#include "raisim_tactile/HybridPenetrationTactile.hpp"
#include "raisim_tactile/MeshSurface.hpp"
#include "raisim_tactile/SensorGrid.hpp"

int main(int argc, char** argv) {
#ifndef RAISIM_TACTILE_EXAMPLE_MESH_PATH
#define RAISIM_TACTILE_EXAMPLE_MESH_PATH "examples/bolt_wrench_minimal/rsc/meshes/wrench_plate.obj"
#endif
  const std::string mesh_path = argc > 1
      ? argv[1]
      : RAISIM_TACTILE_EXAMPLE_MESH_PATH;

  raisim_tactile::MeshSurface mesh;
  if (!mesh.loadObj(mesh_path, 1.0)) {
    std::cerr << "failed to load mesh: " << mesh_path << "\n";
    return 1;
  }

  raisim_tactile::HybridPenetrationTactileConfig cfg;
  cfg.mu = 0.4;
  cfg.protrusion_m = 0.010;
  cfg.vtan_threshold = 1.0e-4;

  const auto left_grid = raisim_tactile::SensorGrid::boltWrenchPad(true, cfg.protrusion_m);
  const auto right_grid = raisim_tactile::SensorGrid::boltWrenchPad(false, cfg.protrusion_m);
  raisim_tactile::HybridPenetrationTactile tactile(cfg, &mesh, &left_grid, &right_grid);

  raisim_tactile::HybridTargetState target;
  raisim_tactile::HybridPadState left_pad;
  left_pad.position_W = Eigen::Vector3d(0.0, -0.04, 0.0);
  left_pad.total_force_W = Eigen::Vector3d(0.0, 0.0, 20.0);

  const auto result = tactile.compute(target, {left_pad}, {});
  const Eigen::VectorXd& flat = result.force_grid_flat;

  int active = 0;
  for (int i = 0; i < 200; ++i) {
    if (flat.segment<3>(i * 3).norm() > 1.0e-9) ++active;
  }

  std::cout << "mesh_vertices=" << mesh.vertexCount()
            << " mesh_triangles=" << mesh.triangleCount()
            << " output_dim=" << flat.size()
            << " active_cells=" << active
            << " force_norm=" << flat.norm()
            << "\n";
  return 0;
}
