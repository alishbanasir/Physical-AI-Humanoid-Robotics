# Example 02: URDF Import to USD

## Overview

This example demonstrates how to import URDF robot files into Isaac Sim's native USD format using the URDF Importer extension. It validates the conversion by checking articulation structure, joint properties, and physics simulation stability.

## Learning Objectives

- Enable URDF Importer extension programmatically
- Convert URDF → USD with custom configuration options
- Validate articulation properties (DOF, joint names, limits)
- Test physics stability after import
- Handle common import errors (missing meshes, invalid syntax)

## Prerequisites

- Isaac Sim 2023.1.1 installed
- URDF robot file with valid syntax
- All mesh files (.stl, .dae, .obj) referenced in URDF must exist

### Validate URDF Before Import

```bash
# Install ROS 2 URDF tools (Ubuntu)
sudo apt install ros-humble-urdf-parser-plugin

# Validate URDF syntax
check_urdf /path/to/humanoid.urdf
```

## Usage

### Basic Import

```bash
cd ~/.local/share/ov/pkg/isaac_sim-2023.1.1
./python.sh /path/to/examples/module-3/chapter-1-isaac-sim/02-urdf-import/main.py \
    --urdf /path/to/humanoid.urdf \
    --output /path/to/humanoid.usd
```

### Advanced Options

```bash
# Fixed-base robot (manipulator arm)
./python.sh main.py \
    --urdf manipulator.urdf \
    --output manipulator.usd \
    --fix-base

# Don't merge fixed joints (preserve URDF structure exactly)
./python.sh main.py \
    --urdf humanoid.urdf \
    --output humanoid.usd \
    --no-merge-fixed

# Disable self-collision (for debugging)
./python.sh main.py \
    --urdf humanoid.urdf \
    --output humanoid.usd \
    --no-self-collision
```

## Expected Output

```
============================================================
URDF to USD Import - Isaac Sim
============================================================

Import configuration:
  fix_base: False
  merge_fixed_joints: True
  self_collision: True
  create_physics_scene: True
  import_inertia_tensor: True

[1/5] Enabling URDF Importer extension...
✓ URDF Importer enabled

[2/5] Importing URDF: /path/to/humanoid.urdf
  Output USD: /path/to/humanoid.usd
✓ URDF imported successfully

[3/5] USD file created: /path/to/humanoid.usd
  File size: 12.34 MB

[3/5] Loading USD into Isaac Sim for validation...

[4/5] Validating articulation structure...
  Degrees of Freedom: 32
  Joint names: left_hip_yaw, left_hip_roll, left_hip_pitch, left_knee, left_ankle_pitch ... (+27 more)
  Initial joint positions: [0.0, 0.0, 0.0]...
  ✓ Joint velocities accessible
  Robot position: [0.000, 0.000, 1.000]
  Robot orientation (quat): [1.000, 0.000, 0.000, 0.000]

✓ Articulation validation complete

[5/5] Testing physics simulation (3.0s)...
  [t=0.00s] Robot Z: 1.000m (deviation: 0.000m)
  [t=0.50s] Robot Z: 0.892m (deviation: 0.108m)
  [t=1.00s] Robot Z: 0.650m (deviation: 0.350m)
  [t=1.50s] Robot Z: 0.512m (deviation: 0.488m)
  [t=2.00s] Robot Z: 0.502m (deviation: 0.498m)
  [t=2.50s] Robot Z: 0.501m (deviation: 0.499m)

  Simulation complete:
    RTF: 2.14x
    Max Z deviation: 0.499m
  ✓ Robot stable (Z deviation < 1.0m)

============================================================
URDF Import Complete!
============================================================

✓ USD file saved: /path/to/humanoid.usd
✓ Robot DOF: 32
✓ Ready for use in Isaac Sim simulations
```

## Key Concepts

### 1. Import Configuration Options

| Option | Default | Description | Use Case |
|--------|---------|-------------|----------|
| `fix_base` | False | Fix base link (immovable) | Manipulator arms, fixed robots |
| `merge_fixed_joints` | True | Merge links connected by fixed joints | Simplify structure, reduce complexity |
| `self_collision` | True | Enable self-collision detection | Humanoids (hand-torso, leg-leg collisions) |
| `create_physics_scene` | True | Auto-configure PhysX scene | Standalone USD files |
| `import_inertia_tensor` | True | Use URDF inertia values | Preserve dynamics from URDF |

### 2. Articulation Validation

The script validates:
- **DOF (Degrees of Freedom)**: Number of actuated joints
- **Joint names**: Naming consistency with URDF
- **Joint positions/velocities**: Accessibility through Isaac Sim API
- **World pose**: Initial position and orientation

### 3. Physics Stability Test

Runs 3-second simulation to detect:
- **Robot explosions**: Large Z-axis deviations (> 1.0m)
- **Jitter**: Excessive vibration or interpenetration
- **PhysX issues**: Insufficient solver iterations

**Stability Criteria**: Z-axis deviation < 1.0m during simulation.

## Troubleshooting

### Issue: "URDF parsing error"

**Cause**: Invalid URDF XML syntax.

**Solution**:
```bash
# Validate URDF
check_urdf /path/to/humanoid.urdf

# Common issues:
# - Missing closing tags (</link>, </joint>)
# - Invalid joint types (typo in "revolute", "prismatic")
# - Duplicate link/joint names
```

### Issue: "Mesh file not found"

**Cause**: URDF references mesh files with incorrect paths.

**Example URDF**:
```xml
<visual>
  <geometry>
    <mesh filename="meshes/base_link.stl"/>  <!-- Relative path -->
  </geometry>
</visual>
```

**Solution**:
1. Ensure mesh files exist at specified paths relative to URDF location
2. Use absolute paths if needed: `<mesh filename="/full/path/to/base_link.stl"/>`
3. Check case sensitivity (Linux: `Base_Link.stl` ≠ `base_link.stl`)

### Issue: Robot explodes during simulation (Z deviation > 1.0m)

**Cause**: PhysX solver iterations insufficient for complex articulations.

**Solution**:
1. Increase solver iterations in Isaac Sim GUI:
   - **Window** → **Physics** → **Settings**
   - Set **Position Iteration Count** to 16-32 (default is 4)
2. Or modify USD file after import (see Example 05: PhysX Configuration)

### Issue: "No joints found (DOF = 0)"

**Cause**: URDF has no movable joints (all joints are `type="fixed"`).

**Solution**:
- Verify URDF has `<joint type="revolute">` or `<joint type="prismatic">` joints
- Check `merge_fixed_joints=True` didn't merge all joints (unlikely but possible)

## File Structure

```
02-urdf-import/
├── main.py          # URDF import script
├── README.md        # This file
└── sample_urdf/     # Sample URDF for testing (optional)
    ├── simple_humanoid.urdf
    └── meshes/
        ├── base_link.stl
        ├── left_leg.stl
        └── right_leg.stl
```

## Modifications

### Experiment 1: Import with Different Configurations

```bash
# Test merging vs not merging fixed joints
./python.sh main.py --urdf robot.urdf --output robot_merged.usd
./python.sh main.py --urdf robot.urdf --output robot_unmerged.usd --no-merge-fixed

# Compare DOF counts (unmerged may have more links)
```

### Experiment 2: Batch Import Multiple URDFs

```python
import glob

urdf_files = glob.glob("/path/to/urdfs/*.urdf")

for urdf_path in urdf_files:
    usd_path = urdf_path.replace(".urdf", ".usd")
    import_urdf_to_usd(urdf_path, usd_path, config)
```

### Experiment 3: Export Import Report

Add to validation function:
```python
import json

report = {
    "urdf_path": args.urdf,
    "usd_path": args.output,
    "dof": validation_results["num_dof"],
    "joint_names": validation_results["joint_names"],
    "rtf": rtf,
    "stable": max_deviation < 1.0
}

with open("import_report.json", "w") as f:
    json.dump(report, f, indent=2)
```

## Next Steps

- **Example 03**: Batch URDF-to-USD conversion for multiple robots
- **Example 04**: Create custom scenes with imported robots
- **Example 05**: Configure PhysX parameters for stable simulation
- **Module 3, Chapter 1, Section 1-3**: Detailed URDF-to-USD conversion guide

## Related Documentation

- [Isaac Sim URDF Importer](https://docs.omniverse.nvidia.com/isaacsim/latest/features/environment_setup/ext_omni_isaac_urdf.html)
- [URDF XML Specification](http://wiki.ros.org/urdf/XML)
- [PhysX ArticulationAPI](https://docs.omniverse.nvidia.com/kit/docs/omni_usd_schema_physics/104.2/class_physx_schema_physx_articulation_a_p_i.html)
