# Codex URDF Conversion Guide

Use this checklist when adapting an arbitrary RaiSim URDF to `raisim-tactile-manager`.

## 1. Identify Tactile Pads

Find the links that should physically touch the target.

Good candidates:

- fingertip pad links,
- fixed child links attached to fingertips,
- links with simple collision boxes/capsules at the contact surface.

Bad candidates:

- visual-only fingertip meshes,
- decorative mesh links with no collision,
- parent links whose frame does not describe the tactile surface.

## 2. Check Collision Geometry

For each tactile pad link, verify that it has a `<collision>` element.

If it does not, add a fixed child link:

```xml
<link name="left_tactile_pad">
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.012 0.08 0.14"/>
    </geometry>
  </collision>
</link>

<joint name="left_fingertip_to_tactile_pad" type="fixed">
  <parent link="left_fingertip"/>
  <child link="left_tactile_pad"/>
  <origin xyz="..." rpy="..."/>
</joint>
```

## 3. Document Pad Frame Convention

Record:

- pad link name,
- local inward/outward normal axis,
- local row axis,
- local column axis,
- grid origin in the pad link frame,
- row/column spacing,
- number of rows and columns,
- protrusion distance.

The module can only distribute force correctly if the grid is meaningful in the pad local frame.

## 4. Resolve Pad Body Indices

Preferred integration:

- resolve URDF link names to RaiSim local body indices in the host application,
- pass those indices to `ContactCollector::collectBodyForce_W(index)`.

Fallback:

- provide explicit local body indices in config when link-name resolution is not available.

## 5. Check Target Mesh

The target needs a triangle mesh for closest-point queries.

Record:

- mesh path,
- mesh scale,
- whether visual and collision geometry use different scales,
- target RaiSim world index for contact filtering.

If face winding is inconsistent, use unsigned distance behavior. The current hybrid model is designed around unsigned distance plus protrusion.

## 6. Smoke Test

Run a short scenario and verify:

- tactile output length is 600 for two 10x10 pads,
- every value is finite,
- active cell count becomes non-zero during contact,
- left/right cells match expected contact side,
- no reward or PPO observation code was changed while adapting the URDF.

## 7. Keep Responsibilities Separate

Do not put these concerns into the tactile module:

- reinforcement learning reward,
- PPO observation layout decisions,
- OpenCV/Unity visualization,
- task-specific success logic.

The host environment owns those policies. The tactile module only owns contact collection, mesh query, grid geometry, and force distribution.
