# Tactile Manager Modularization Plan

Status: in progress, 2026-04-25

Working branch/worktree:

- branch: `tactile-manager-modularization`
- worktree: `envs/bolt_wrench_grasp_iteration/.codex/worktrees/tactile-manager-modularization`
- original v0.0.1 training worktree is intentionally left untouched by this refactor.

## Goal

현재 `bolt_wrench_grasp_iteration` 안에 섞여 있는 tactile contact 수집, hybrid penetration 분배, tactile geometry, URDF 요구사항을 분리해서 RaiSim 사용자가 다음처럼 곧바로 재사용할 수 있는 독립 repo로 만든다.

```bash
git clone <tactile-manager-repo>
cmake -S . -B build -DCMAKE_PREFIX_PATH=<raisim-install>
cmake --build build
```

최종 repo는 `raisimLib` checkout 안이나 옆에 clone해서 사용할 수 있어야 하며, 특정 PPO runner, 특정 wrench task, 특정 observation layout에 묶이지 않아야 한다.

## Current Code Map

현재 tactile 구현은 두 층으로 나뉘어 있다.

- `rsc/tactile/GQGITactileManager.*`
  - RaiSim contact solver에서 EE-target contact impulse를 수집한다.
  - aggregate와 bounded contact sample flat buffer를 제공한다.
  - 비교적 모듈화되어 있지만 `Yaml.hpp`, GQGI namespace, 환경 config 구조에 묶여 있다.
- `rsc/Environment/Environment.hpp`
  - `hybrid_tactile` config parsing
  - target mesh OBJ load
  - sensing cell grid 생성
  - target/object pose 변환
  - RaiSim contact impulse를 100 cells x 3D x 2 sides = 600-dim force grid로 분배
  - contact point visualization용 point 계산
  - observation/reward/grasp-quality와 직접 연결

외부 repo로 빼야 할 핵심은 `Environment.hpp` 안의 hybrid tactile engine이다. 현재 `GQGITactileManager`는 contact sample manager로 남길지, hybrid engine의 입력 어댑터로 통합할지 선택해야 한다.

## Non-Goals

- v0.0.1/v0.1 runner, tester, log infrastructure 변경 없음.
- 현재 학습 reward tuning 변경 없음.
- PPO observation layout 변경 없음.
- RaiSim solver 자체 수정 없음.
- OpenCV/Unity visualizer를 core library에 넣지 않음. 시각화는 optional example 또는 adapter로 둔다.

## Target Repository Shape

권장 repo 이름: `raisim-tactile-manager`

```text
raisim-tactile-manager/
  CMakeLists.txt
  README.md
  LICENSE
  include/raisim_tactile/
    ContactCollector.hpp
    HybridPenetrationTactile.hpp
    MeshSurface.hpp
    SensorGrid.hpp
    TactileTypes.hpp
    UrdfSensorSpec.hpp
  src/
    ContactCollector.cpp
    HybridPenetrationTactile.cpp
    MeshSurface.cpp
    SensorGrid.cpp
  examples/
    bolt_wrench_minimal/
      CMakeLists.txt
      main.cpp
      rsc/
        end_effector_tactile_example.urdf
        wrench_example.urdf
        meshes/
          wrench.obj
      config.yaml
  docs/
    urdf_requirements.md
    codex_urdf_conversion_guide.md
    raiSim_integration.md
    coordinate_conventions.md
  tests/
    test_mesh_surface.cpp
    test_sensor_grid.cpp
    test_force_distribution.cpp
```

## Proposed Module Boundaries

### 1. `TactileTypes`

Pure data structs only.

- `ContactSample`
- `ContactFrameSample`
- `TactileGridConfig`
- `HybridTactileConfig`
- `TactileFrameConvention`
- output buffers:
  - aggregate impulse
  - per-contact samples
  - per-cell force grid
  - optional per-cell contact points

This layer must not include `Yaml.hpp`, environment reward code, or Python bindings.

### 2. `ContactCollector`

RaiSim-specific adapter.

Responsibilities:

- Read `raisim::ArticulatedSystem::getContacts()`.
- Filter contacts by target world index.
- Filter contacts by tactile pad local body indices.
- Convert contact impulse from contact frame to world frame.
- Normalize direction convention.
- Return finite, bounded contact samples.

Inputs should be explicit:

- `raisim::ArticulatedSystem* tactile_body`
- `std::size_t target_world_index`
- `std::vector<int> pad_body_indices`
- `double simulation_dt`

The collector should not know about 10x10 grids or wrench meshes.

### 3. `MeshSurface`

Geometry-only mesh query module.

Responsibilities:

- Load OBJ triangle mesh.
- Apply mesh scale.
- Query closest point, unsigned distance, face normal.
- Keep the current unsigned penetration behavior documented because the canonical plan explains why signed distance was avoided.

No RaiSim dependency is required here except optional helper conversion types. Prefer Eigen-only implementation.

### 4. `SensorGrid`

Geometry-only tactile cell layout.

Responsibilities:

- Build rectangular grid cells from dimensions, resolution, local frame offset, and side convention.
- Support the current 10x10 left/right pad layout as a preset.
- Allow arbitrary URDF link names/body indices through config.

This replaces hard-coded constants such as:

- `x_left = 0.01091 + protrusion`
- `x_right = -(0.01091 + protrusion)`
- row/column spacing
- local pad body indices

### 5. `HybridPenetrationTactile`

Policy/mechanism boundary for force distribution.

Responsibilities:

- Take collector samples, pad poses/velocities, target pose/velocity, mesh query, and sensor grid.
- Compute penetration weights per cell.
- Distribute RaiSim contact force over active cells.
- Add shear term from tangential relative velocity.
- Return 600-dim compatible force grid when using two 10x10 pads.

This class should not read YAML, access PPO observations, compute reward, or update visualizers.

## Public API Sketch

```cpp
raisim_tactile::HybridTactileConfig cfg;
cfg.mu = 0.4;
cfg.protrusion_m = 0.003;
cfg.vtan_threshold = 1.0e-4;

raisim_tactile::MeshSurface target_mesh;
target_mesh.loadObj("rsc/meshes/wrench.obj", target_mesh_scale);

raisim_tactile::SensorGrid left_grid = SensorGrid::rectangularPad(...);
raisim_tactile::SensorGrid right_grid = SensorGrid::rectangularPad(...);

raisim_tactile::ContactCollector collector(
    end_effector,
    target->getIndexInWorld(),
    {left_pad_body_idx, right_pad_body_idx},
    simulation_dt);

raisim_tactile::HybridPenetrationTactile tactile(cfg, target_mesh, left_grid, right_grid);

collector.collect();
tactile.compute({
    .contacts = collector.samples(),
    .pad_states = current_pad_states,
    .target_state = current_target_state,
});

Eigen::VectorXd flat = tactile.forceGridFlat();
```

## URDF Requirements

The example URDF and documentation must make these requirements explicit.

Required:

- Tactile pad links must be stable RaiSim local bodies.
- Each tactile side must expose either:
  - known link names that can be resolved to local body indices, or
  - explicit local body indices supplied by config.
- Pad link frame convention must be documented:
  - local pad axes
  - inward normal direction
  - grid origin
  - row/column axis directions
- Target object must have a mesh usable for closest-point queries.
- Target visual/collision scale mismatch must be configurable and documented.
- Collision contacts must occur between tactile pad bodies and target object.

Recommended:

- Use simple collision geometry for tactile pad bodies.
- Keep tactile pad collision separate from decorative visual meshes.
- Use visual bodies only for debug rendering, not as the source of physical contact.

## Codex URDF Conversion Guide

Create `docs/codex_urdf_conversion_guide.md` in the external repo. It should instruct an agent to:

1. Identify intended tactile pad links in the URDF.
2. Check whether those links have collision geometry.
3. Add or adjust fixed child pad links if tactile pads are only visual meshes.
4. Ensure the pad link frame has a documented inward normal and grid axes.
5. Add config entries mapping `left_pad_link`, `right_pad_link`, target mesh path, mesh scale, grid size, resolution, and protrusion.
6. Run the minimal example smoke test and inspect active cell count plus finite force grid output.

The guide should avoid task-specific reward/PPO instructions.

## Migration Phases

### Phase 0: Freeze Current Behavior

Deliverables:

- Add a local smoke/diagnostic command that records current hybrid tactile output from this repo.
- Capture expected invariants:
  - output length 600
  - finite values
  - left/right active cell counts
  - no non-finite observation values
  - current v0.0.1 runner still builds

This phase protects the current learning setup before moving code.

Current result:

- Added `tester/etc/scripts/smoke_tactile_modularization_baseline.py`.
- Baseline before engine extraction:
  - command: `PYTHONPATH=<worktree-root> python3 tester/etc/scripts/smoke_tactile_modularization_baseline.py --steps 30 --out-dir logs/tests/tactile_modularization_baseline_latest`
  - output length: 600
  - finite: true
  - best active cells: 6
  - left/right active cells: 0 / 6
  - max force norm: 1122.7156063522743
  - sum force norm: 3802.68290358929

### Phase 1: Local Internal Extraction

Deliverables inside this repo:

- Move mesh query and grid generation out of `Environment.hpp` into local tactile files.
- Keep public behavior unchanged.
- `Environment.hpp` should become an adapter that passes RaiSim object states into the tactile module.

Expected changed files:

- `rsc/tactile/HybridPenetrationTactile.hpp`
- `rsc/tactile/HybridPenetrationTactile.cpp`
- `rsc/tactile/MeshSurface.hpp`
- `rsc/tactile/MeshSurface.cpp`
- `rsc/tactile/SensorGrid.hpp`
- `rsc/tactile/SensorGrid.cpp`
- `rsc/Environment/Environment.hpp`
- `CMakeLists.txt`

Current result:

- Added `rsc/tactile/MeshSurface.*`.
- Added `rsc/tactile/SensorGrid.*`.
- Added `rsc/tactile/HybridPenetrationTactile.*`.
- `Environment.hpp` now keeps RaiSim orchestration and passes target/pad state into the tactile module.
- Verification after engine extraction:
  - `cmake --build build -j 8`: pass
  - `smoke_tactile_modularization_baseline.py --steps 30`: pass
  - best raw 600-d vector max absolute difference vs pre-extraction baseline: `9.0949470177292824e-13`

### Phase 2: Remove Environment-Specific Assumptions

Deliverables:

- Replace hard-coded pad body indices with link-name or config-driven indices.
- Replace hard-coded 10x10 grid constants with `SensorGridConfig`.
- Replace wrench-specific naming with target/object-neutral naming.
- Preserve the current bolt-wrench config as one preset.

### Phase 3: Create External Repo Skeleton

Deliverables:

- Create standalone CMake project.
- Export headers under `include/raisim_tactile`.
- Link against RaiSim and Eigen.
- Build library target `raisim_tactile`.
- Add examples and docs listed above.

This should happen after Phase 1 and Phase 2 are proven locally to reduce the chance of exporting unstable boundaries.

### Phase 4: Port Current Implementation

Deliverables:

- Copy extracted local modules into the external repo.
- Replace local `Yaml.hpp` usage with plain config structs plus optional YAML helper.
- Keep the library usable without Python.
- Add a minimal RaiSim example that creates world, loads example URDFs, steps simulation, and prints tactile grid summary.

### Phase 5: Reintegrate This Environment Through the External Module

Deliverables:

- Add external repo as either:
  - sibling CMake dependency via `CMAKE_PREFIX_PATH`, or
  - git submodule only if explicitly desired later.
- In this environment, remove duplicated hybrid tactile implementation after dependency is working.
- Keep `getRawTactileFlat()` and observation shape unchanged.

### Phase 6: Verification

Required checks:

- Standalone repo builds from clean clone.
- Example URDF simulation runs.
- This environment builds against external module.
- `runner_v0.0.1` smoke runs with `--no-train-unity --no-train-opencv-tactile`.
- Visual tester still receives 600-dim tactile values.
- Numeric comparison against Phase 0 baseline stays within expected tolerance.

## Compatibility Rules

- Keep current output layout: left 100 cells x xyz, then right 100 cells x xyz.
- Keep world-frame force vector convention.
- Keep force units consistent with current behavior: impulse divided by `simulation_dt` for hybrid tactile force distribution.
- Keep finite output even when no contacts or mesh query fails.
- Do not make OpenCV or Unity a core dependency.
- Keep mesh closest-point code deterministic and free of global state.

## Main Risks

- RaiSim local body index resolution differs across URDFs. Link-name resolution or explicit config fallback is required.
- Current mesh scale handling depends on known target visual/collision mismatch. External repo must expose this as config.
- Signed distance is not reliable for current wrench mesh. Keep unsigned-distance penetration as the default unless mesh winding is validated.
- Pulling code out while training is running could create confusion. Refactor should be committed separately from ongoing v0.0.1 reward/log changes.

## Immediate Next Steps

1. Add Phase 0 tactile baseline smoke in this repo.
2. Extract `MeshSurface` and `SensorGrid` locally without changing behavior.
3. Extract `HybridPenetrationTactile` locally and make `Environment.hpp` only orchestrate RaiSim state.
4. Build and run the Phase 0 smoke again.
5. Only after the local boundary is stable, create the external standalone repo.

## Canonical Doc Relationship

`docs/canonical/02_tactile_hybrid_penetration_plan.md` remains the behavior-level canonical reference for the current hybrid penetration algorithm. This document is the engineering migration plan for turning that behavior into a reusable module/repo. If implementation details change during extraction, update the canonical document only for behavior changes, not for file movement.
