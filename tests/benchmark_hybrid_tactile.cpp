#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#include "raisim_tactile/HybridPenetrationTactile.hpp"
#include "raisim_tactile/MeshSurface.hpp"
#include "raisim_tactile/SensorGrid.hpp"

namespace {

void writeGridPlaneObj(const std::string& path, int segments) {
  std::ofstream out(path);
  const double lo = -0.1;
  const double hi = 0.1;
  for (int y = 0; y <= segments; ++y) {
    for (int x = 0; x <= segments; ++x) {
      const double px = lo + (hi - lo) * static_cast<double>(x) / segments;
      const double py = lo + (hi - lo) * static_cast<double>(y) / segments;
      out << "v " << px << " " << py << " 0.0\n";
    }
  }
  out << "vn 0.0 0.0 1.0\n";
  const auto vid = [segments](int x, int y) {
    return y * (segments + 1) + x + 1;
  };
  for (int y = 0; y < segments; ++y) {
    for (int x = 0; x < segments; ++x) {
      const int v00 = vid(x, y);
      const int v10 = vid(x + 1, y);
      const int v01 = vid(x, y + 1);
      const int v11 = vid(x + 1, y + 1);
      out << "f " << v00 << "//1 " << v10 << "//1 " << v11 << "//1\n";
      out << "f " << v00 << "//1 " << v11 << "//1 " << v01 << "//1\n";
    }
  }
}

struct BenchResult {
  double millis = 0.0;
  double force_norm_accum = 0.0;
  std::size_t mesh_queries = 0;
};

BenchResult runBench(const raisim_tactile::HybridPenetrationTactile& tactile,
                     const raisim_tactile::HybridTargetState& target,
                     const raisim_tactile::HybridPadState& pad,
                     int iterations) {
  BenchResult out;
  const auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < iterations; ++i) {
    const auto result = tactile.compute(target, {pad}, {});
    out.force_norm_accum += result.force_grid_flat.norm();
    out.mesh_queries += result.mesh_query_count;
  }
  const auto t1 = std::chrono::steady_clock::now();
  out.millis = std::chrono::duration<double, std::milli>(t1 - t0).count();
  return out;
}

}  // namespace

int main(int argc, char** argv) {
  const int iterations = argc > 1 ? std::max(1, std::atoi(argv[1])) : 10000;
  const std::string mesh_path = "benchmark_grid_plane.obj";
  writeGridPlaneObj(mesh_path, 20);

  raisim_tactile::MeshSurface mesh;
  if (!mesh.loadObj(mesh_path, 1.0)) {
    std::cerr << "failed to load benchmark mesh\n";
    return 1;
  }

  raisim_tactile::HybridPenetrationTactileConfig full_cfg;
  full_cfg.protrusion_m = 0.010;
  full_cfg.use_contact_hint_sparse_cells = false;

  raisim_tactile::HybridPenetrationTactileConfig sparse_cfg = full_cfg;
  sparse_cfg.use_contact_hint_sparse_cells = true;
  sparse_cfg.max_cells_per_contact_hint = 25;

  const auto left_grid = raisim_tactile::SensorGrid::boltWrenchPad(true, full_cfg.protrusion_m);
  const auto right_grid = raisim_tactile::SensorGrid::boltWrenchPad(false, full_cfg.protrusion_m);

  raisim_tactile::HybridTargetState target;
  raisim_tactile::HybridPadState pad;
  pad.position_W = Eigen::Vector3d(0.0, -0.04, 0.0);
  pad.total_force_W = Eigen::Vector3d(0.0, 0.0, 10.0);
  pad.contact_points_W.push_back(pad.position_W + pad.orientation_W * left_grid.cellsLocal()[54]);

  raisim_tactile::HybridPenetrationTactile full_tactile(
      full_cfg, &mesh, &left_grid, &right_grid);
  raisim_tactile::HybridPenetrationTactile sparse_tactile(
      sparse_cfg, &mesh, &left_grid, &right_grid);

  (void)runBench(full_tactile, target, pad, 100);
  (void)runBench(sparse_tactile, target, pad, 100);

  const BenchResult full = runBench(full_tactile, target, pad, iterations);
  const BenchResult sparse = runBench(sparse_tactile, target, pad, iterations);
  const double full_us = 1000.0 * full.millis / iterations;
  const double sparse_us = 1000.0 * sparse.millis / iterations;

  std::cout << "mesh_triangles=" << mesh.triangleCount()
            << " iterations=" << iterations
            << " full_us_per_compute=" << full_us
            << " sparse_us_per_compute=" << sparse_us
            << " speedup=" << (full_us / std::max(sparse_us, 1.0e-12))
            << " full_queries_per_compute="
            << (static_cast<double>(full.mesh_queries) / iterations)
            << " sparse_queries_per_compute="
            << (static_cast<double>(sparse.mesh_queries) / iterations)
            << " full_force_accum=" << full.force_norm_accum
            << " sparse_force_accum=" << sparse.force_norm_accum
            << "\n";
  return 0;
}
