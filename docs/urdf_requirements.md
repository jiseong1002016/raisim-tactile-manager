# Tactile Manager URDF Requirements

Status: draft for the future standalone tactile manager repo.

## Purpose

The hybrid penetration tactile manager needs RaiSim contact impulses from physical pad bodies and a mesh surface for geometric force distribution. A URDF does not need to be designed only for this project, but it must expose enough structure for the tactile module to identify pad bodies, contact geometry, and target mesh scale.

## Required Robot / End-Effector Conditions

Each tactile side must have at least one RaiSim body that can physically contact the target.

Required per side:

- A stable URDF link for the tactile pad or fingertip.
- Collision geometry on that link or on a fixed child link.
- A known local frame convention:
  - one local axis points approximately toward the target/contact surface,
  - two local axes span the tactile grid rows and columns,
  - the grid origin and extents can be described in the link frame.
- A way to resolve the pad body:
  - preferred: link name resolved to RaiSim local body index,
  - fallback: explicit local body index in config.

Recommended:

- Keep tactile collision geometry simple, such as box or capsule.
- Keep decorative visual meshes separate from tactile collision bodies.
- Avoid relying on visual-only bodies for contact force.
- Use fixed joints for added tactile pad links if the original URDF has only decorative fingertip meshes.

## Required Target Object Conditions

The target object must provide:

- A RaiSim body that receives contacts from the tactile pads.
- A triangle mesh usable for closest-point queries.
- A documented mesh scale.
- A documented relationship between visual mesh scale and collision mesh scale.

For the current bolt-wrench task, the tactile module uses the target mesh loaded from the target URDF visual mesh unless a tactile mesh override is supplied.

## Config Fields Needed By A Standalone Module

Minimum robot-side fields:

```yaml
tactile:
  left_pads:
    - link: ee_3_left
    - link: ee_4_left
  right_pads:
    - link: ee_3_right
    - link: ee_4_right
  grid:
    rows: 10
    cols: 10
    local_origin: [0.01091, 0.003750, -0.060840]
    row_axis: [0.0, 1.0, 0.0]
    col_axis: [0.0, 0.0, 1.0]
    row_spacing: 0.007500
    col_spacing: 0.013520
    protrusion_m: 0.003
```

Minimum target-side fields:

```yaml
target:
  body: target_object
  mesh: wrench/wrench/meshes/wrench.obj
  mesh_scale: 1.0
```

Minimum physics fields:

```yaml
hybrid_penetration:
  mu: 0.4
  vtan_threshold: 1.0e-4
```

## Codex Agent Conversion Checklist

When adapting an arbitrary URDF for this tactile module:

1. Find the fingertip or pad links intended to contact the target.
2. Check whether those links have collision geometry, not only visual geometry.
3. If no physical pad exists, add a fixed child link with simple collision geometry at the tactile surface.
4. Record the pad link names.
5. Determine the local frame axes for the tactile grid.
6. Add or generate a config entry with grid origin, axes, spacing, resolution, and protrusion.
7. Find the target mesh path and scale used by RaiSim.
8. Run a smoke test that confirms:
   - the module resolves the pad links,
   - contacts occur between pads and target,
   - the 600-d output is finite,
   - active cell counts become non-zero during contact.

Do not change reward, PPO observation layout, or visualizer code while doing a URDF conversion unless the task explicitly asks for that.
