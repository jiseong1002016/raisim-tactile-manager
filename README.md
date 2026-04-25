# raisim-tactile-manager

Reusable tactile sensing utilities for RaiSim contact-based manipulation tasks.

The current module implements a hybrid penetration tactile model:

- RaiSim contact solver supplies physical contact impulse/force.
- A target mesh closest-point query estimates which tactile cells are geometrically active.
- The contact force is distributed over a tactile grid and returned as a flat force vector.

The default compatible layout is:

```text
left  pad: 100 cells x [Fx, Fy, Fz] = flat[0:300]
right pad: 100 cells x [Fx, Fy, Fz] = flat[300:600]
```

Forces are world-frame vectors.

## Build

From a checkout next to, or inside, a RaiSim installation:

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=/home/Jiseong/raisim_ws/raisimlib/raisim/linux
cmake --build build -j
ctest --test-dir build --output-on-failure
```

If RaiSim is already discoverable by CMake, `CMAKE_PREFIX_PATH` can be omitted.

## Minimal Example

```bash
./build/raisim_tactile_bolt_wrench_minimal
```

The example uses a synthetic mesh and pad state so it can verify the tactile engine without starting a full simulator.

## Main Modules

- `MeshSurface`
  - Loads OBJ triangle meshes.
  - Provides closest-point, distance, and face-normal queries.
- `SensorGrid`
  - Builds local tactile cell positions.
  - Currently includes the bolt-wrench 10x10 pad preset.
- `HybridPenetrationTactile`
  - Distributes pad contact force over active tactile cells.
  - Computes normal and shear components.
- `ContactCollector`
  - RaiSim adapter for reading `ArticulatedSystem::getContacts()`.
  - Converts solver impulses into world-frame samples and per-body force.

## Integration Sketch

```cpp
raisim_tactile::MeshSurface mesh;
mesh.loadObj("target.obj", mesh_scale);

auto left_grid = raisim_tactile::SensorGrid::boltWrenchPad(true, 0.003);
auto right_grid = raisim_tactile::SensorGrid::boltWrenchPad(false, 0.003);

raisim_tactile::HybridPenetrationTactileConfig cfg;
cfg.mu = 0.4;
cfg.protrusion_m = 0.003;

raisim_tactile::HybridPenetrationTactile tactile(cfg, &mesh, &left_grid, &right_grid);

raisim_tactile::HybridTargetState target;
std::vector<raisim_tactile::HybridPadState> left_pads;
std::vector<raisim_tactile::HybridPadState> right_pads;

auto result = tactile.compute(target, left_pads, right_pads);
Eigen::VectorXd flat_600 = result.force_grid_flat;
```

## URDF Requirements

See [docs/urdf_requirements.md](docs/urdf_requirements.md) and [docs/codex_urdf_conversion_guide.md](docs/codex_urdf_conversion_guide.md).

Short version:

- tactile pads must be physical RaiSim bodies with collision geometry,
- pad link frames must have documented local grid axes,
- the target must provide a mesh and mesh scale for closest-point queries,
- visual-only fingertip meshes are not enough for contact-force collection.

## Status

This repository was extracted from `bolt_wrench_grasp_iteration` after validating that the extracted engine reproduces the original 600-d tactile output with max absolute difference around `1e-12` on the baseline smoke.
