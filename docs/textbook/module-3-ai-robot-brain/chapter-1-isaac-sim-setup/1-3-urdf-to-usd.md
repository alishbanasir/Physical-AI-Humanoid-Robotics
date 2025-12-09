---
title: "1.3 URDF to USD Conversion"
description: "Convert URDF humanoid robots to Isaac Sim USD format while preserving articulation, physics properties, and collision geometry"
---

# 1.3 URDF to USD Conversion

## Introduction

URDF (Unified Robot Description Format) from Module 1 describes robot kinematics and dynamics in XML. Isaac Sim uses **USD (Universal Scene Description)** as its native format. This section teaches you to convert URDF → USD using Isaac Sim's **URDF Importer extension**, preserving all articulation properties, physics materials, and collision geometry.

---

## URDF vs USD: Key Differences

| Aspect | URDF (XML) | USD |
|--------|------------|-----|
| **Purpose** | Robot-specific kinematics/dynamics | General 3D scene description |
| **Scope** | Links, joints, sensors, transmissions | Meshes, materials, lights, cameras, physics, robots |
| **Physics** | Basic (mass, inertia, friction) | Advanced (PhysX articulations, contacts, drives) |
| **Visualization** | Simple materials, no PBR | PBR materials, RTX lighting |
| **Collaboration** | Single file | Layered composition, references |

**Why Convert**: Isaac Sim's PhysX physics and RTX rendering require USD. URDF Importer bridges Module 1 robots to Isaac Sim.

---

## Step 1: Enable URDF Importer Extension

1. Launch Isaac Sim
2. Go to **Window** → **Extensions**
3. Search for **"URDF Importer"**
4. Toggle switch to **ON** (extension loads automatically)

**Screenshot Placeholder**: `urdf-importer-extension-panel.png` - Extensions window with URDF Importer enabled

---

## Step 2: Import URDF via GUI

### Method A: GUI Import (Interactive)

1. In Isaac Sim, go to **Isaac Utils** → **URDF Importer**
2. URDF Importer window opens
3. Click **"Select URDF File"** → Browse to your Module 1 humanoid URDF (e.g., `humanoid.urdf`)
4. Configure import settings:
   - **Import Root Link**: Usually `base_link` or `pelvis`
   - **Fix Base Link**: ☐ Unchecked (humanoid is mobile, not fixed like manipulator arm)
   - **Merge Fixed Joints**: ☑ Checked (reduces complexity for links connected by fixed joints)
   - **Self Collision**: ☑ Checked (enable humanoid self-collision detection, e.g., hand touching torso)
   - **Create Physics Scene**: ☑ Checked (auto-configure PhysX gravity and solver)
5. Click **"Import"**
6. Wait for processing (5-30 seconds for humanoid with 30+ links)
7. Robot appears in viewport at origin (0,0,0)

**Screenshot Placeholder**: `urdf-importer-settings.png` - URDF Importer window with configuration options

### Method B: Python API Import (Automated)

```python
import omni.isaac.core.utils.stage as stage_utils
from omni.isaac.core.utils.extensions import enable_extension

# Enable URDF Importer extension
enable_extension("omni.importer.urdf")

from omni.importer.urdf import _urdf

# Import URDF to USD
urdf_path = "/path/to/humanoid.urdf"
usd_output_path = "/path/to/humanoid.usd"

# Import with configuration
_urdf.acquire_urdf_interface().import_robot(
    urdf_path,                     # Input URDF file
    usd_output_path,              # Output USD file
    fix_base=False,               # Humanoid is mobile
    merge_fixed_joints=True,      # Simplify structure
    self_collision=True,          # Enable self-collision
    create_physics_scene=True,    # Auto-configure PhysX
    import_inertia_tensor=True    # Use URDF inertia values
)

print(f"URDF imported to {usd_output_path}")
```

---

## Step 3: Inspect Articulation Properties

After import, inspect the USD articulation structure:

### USD Stage Hierarchy

```
/World/humanoid                    # Root articulation prim
├── /base_link                     # Root link (pelvis)
│   ├── Physics: RigidBodyAPI      # PhysX rigid body
│   ├── Mass: 8.5 kg               # From URDF <mass> tag
│   ├── Inertia: [Ixx, Iyy, Izz]  # From URDF <inertia> tag
│   └── Collision: Convex mesh     # From URDF <collision> geometry
├── /left_hip_yaw_joint            # Revolute joint
│   ├── Physics: ArticulationAPI   # PhysX joint
│   ├── Joint Type: Revolute       # From URDF <joint type="revolute">
│   ├── Axis: [0, 0, 1]            # Rotation axis (Z-axis for yaw)
│   ├── Limits: [-0.5, 0.5] rad    # From URDF <limit lower/upper>
│   ├── Damping: 0.1               # From URDF <dynamics damping>
│   ├── Stiffness: 0.0             # From URDF <dynamics friction>
│   └── Max Effort: 100 N·m        # From URDF <limit effort>
├── /left_hip_yaw_link             # Child link
│   └── ...                        # Similar structure
└── ...                            # All other joints and links
```

### Property Panel Inspection

1. Select robot in viewport
2. Open **Property** panel (right side)
3. Click on individual links/joints to inspect:
   - **Physics → RigidBody**: Mass, center of mass, inertia tensor
   - **Physics → Collision**: Collision shapes (convex, mesh, sphere)
   - **Physics → ArticulationJoint**: Joint type, axis, limits, damping, stiffness

**Screenshot Placeholder**: `usd-stage-hierarchy.png` - Stage window showing humanoid articulation tree

---

## Step 4: Configure Articulation Drives

URDF `<transmission>` tags are limited. Isaac Sim uses **PhysX Drives** for joint control (more powerful).

### Drive Types

| Drive Type | Description | Use Case |
|------------|-------------|----------|
| **Position** | PD controller tracking target position | Joint position control (e.g., inverse kinematics) |
| **Velocity** | PD controller tracking target velocity | Velocity control (e.g., wheeled robots) |
| **Force/Torque** | Direct force/torque application | Torque control (e.g., RL training, dynamic control) |

### Configure Drive via Python

```python
import omni.isaac.core.utils.prims as prim_utils
from pxr import PhysxSchema, UsdPhys

# Get joint prim path
joint_path = "/World/humanoid/left_hip_yaw_joint"
joint_prim = prim_utils.get_prim_at_path(joint_path)

# Get or create Drive API
drive_api = UsdPhys.DriveAPI.Apply(joint_prim, "angular")  # "angular" for revolute, "linear" for prismatic

# Configure PD gains for position drive
drive_api.GetStiffnessAttr().Set(1000.0)  # P gain (N·m/rad)
drive_api.GetDampingAttr().Set(100.0)      # D gain (N·m·s/rad)
drive_api.GetMaxForceAttr().Set(100.0)     # Max torque limit (N·m)

# Set target position (example: 0.2 radians)
drive_api.GetTargetPositionAttr().Set(0.2)
```

**Typical Humanoid Drive Gains**:
- **Hip/Knee joints**: Stiffness 1000-5000, Damping 50-200 (strong for weight support)
- **Ankle joints**: Stiffness 500-2000, Damping 20-100 (moderate for balance)
- **Arm joints**: Stiffness 100-500, Damping 10-50 (light for manipulation)

---

## Step 5: Configure Physics Materials

URDF `<mu1>` and `<mu2>` tags specify friction. Isaac Sim uses **PhysicsMaterialAPI** for richer material properties.

### Apply Physics Material to Links

```python
from pxr import UsdShade, UsdPhys

# Create physics material
stage = omni.usd.get_context().get_stage()
material_path = "/World/Materials/RobotMaterial"
material = UsdShade.Material.Define(stage, material_path)

# Configure friction and restitution
physics_material = UsdPhys.MaterialAPI.Apply(material.GetPrim())
physics_material.CreateStaticFrictionAttr(0.7)   # Static friction (0.6-0.9 for rubber-like feet)
physics_material.CreateDynamicFrictionAttr(0.6)  # Dynamic friction
physics_material.CreateRestitutionAttr(0.0)      # Restitution (bounciness: 0=no bounce, 1=perfect bounce)

# Bind material to robot links
link_path = "/World/humanoid/base_link"
link_prim = prim_utils.get_prim_at_path(link_path)
UsdShade.MaterialBindingAPI(link_prim).Bind(material)
```

**Material Properties for Humanoid**:
- **Feet**: High friction (0.7-0.9 static, 0.6-0.8 dynamic), low restitution (0.0-0.1) for stable walking
- **Hands**: Medium friction (0.5-0.7) for grasping
- **Torso/Legs**: Low friction (0.3-0.5) to reduce unwanted sticking

---

## Step 6: Validate Conversion

### Validation Checklist

- [ ] **Joint Limits**: All revolute joints have correct lower/upper limits from URDF
- [ ] **Collision Geometry**: Collision meshes visible when **Show → Physics → Colliders** enabled
- [ ] **Mass Properties**: Link masses match URDF values (check Property panel)
- [ ] **Inertia Tensors**: Inertia diagonal elements [Ixx, Iyy, Izz] non-zero and reasonable
- [ ] **Joint Axes**: Joint rotation axes correct (visualize with **Show → Physics → Joint Axes**)
- [ ] **Self-Collision**: Robot links don't interpenetrate when joints move to limits

### Test Simulation Stability

1. Click **Play** (▶️) to start simulation
2. Robot should fall to ground gracefully under gravity (not explode or jitter)
3. Manually move joints using **"Joint Inspector"** (**Isaac Utils** → **Joint Inspector**)
4. Verify joints respect limits (cannot exceed specified ranges)
5. Check RTF ≥1.0 in headless mode (see Section 1-2 for RTF measurement)

**If robot explodes or jitters**: See Section 1-2 Troubleshooting Issue 6 (increase PhysX solver iterations to 16-32)

**Screenshot Placeholder**: `isaac-sim-humanoid-validation.png` - Humanoid standing in Isaac Sim after URDF import

---

## Step 7: Save USD File

### Save via GUI

1. **File** → **Save** or **Save As**
2. Choose output path (e.g., `/home/user/robots/humanoid_isaac.usd`)
3. File saved in binary USD format (`.usd`) for fast loading

### Save via Python

```python
import omni.usd

# Get stage
stage = omni.usd.get_context().get_stage()

# Export to USD file
output_path = "/home/user/robots/humanoid_isaac.usd"
stage.Export(output_path)
print(f"USD exported to {output_path}")
```

---

## Batch URDF-to-USD Conversion Script

For converting multiple robots, create automation script:

```python
import os
from omni.importer.urdf import _urdf

# Input directory with URDF files
urdf_dir = "/path/to/urdf_robots/"
usd_output_dir = "/path/to/usd_robots/"

# Convert all URDF files in directory
for filename in os.listdir(urdf_dir):
    if filename.endswith(".urdf"):
        urdf_path = os.path.join(urdf_dir, filename)
        usd_filename = filename.replace(".urdf", ".usd")
        usd_path = os.path.join(usd_output_dir, usd_filename)

        print(f"Converting {filename} → {usd_filename}")

        _urdf.acquire_urdf_interface().import_robot(
            urdf_path,
            usd_path,
            fix_base=False,
            merge_fixed_joints=True,
            self_collision=True,
            create_physics_scene=False  # Don't create physics scene for batch (add manually later)
        )

print(f"Batch conversion complete: {len(os.listdir(usd_output_dir))} USD files created")
```

See **Example 03-urdf-to-usd-batch** in code examples for full script.

---

## Common Issues and Solutions

### Issue: URDF Import Fails with "Parsing Error"

**Cause**: Invalid URDF XML syntax or missing mesh files referenced in `<mesh filename="...">`

**Solution**:
1. Validate URDF with `check_urdf` tool (ROS 2):
   ```bash
   check_urdf /path/to/humanoid.urdf
   ```
2. Ensure all mesh files (.stl, .dae, .obj) exist at paths specified in URDF
3. Use absolute paths or ensure relative paths are correct

### Issue: Converted Robot Has No Collision

**Cause**: URDF `<collision>` geometry missing or `<geometry>` tags empty

**Solution**:
1. Add `<collision>` tags to all links in URDF with simplified geometry (e.g., boxes, cylinders, or convex meshes)
2. Reimport URDF to USD

### Issue: Joint Limits Not Respected

**Cause**: PhysX joint limits not applied during import

**Solution**:
1. Check joint has limits in Property panel (**Physics → ArticulationJoint → Limits**)
2. If missing, manually set via Python:
   ```python
   joint = prim_utils.get_prim_at_path("/World/humanoid/left_hip_yaw_joint")
   joint_api = UsdPhys.RevoluteJoint(joint)
   joint_api.CreateLowerLimitAttr(-0.5)  # radians
   joint_api.CreateUpperLimitAttr(0.5)
   ```

---

## Key Takeaways

1. **URDF Importer extension** converts URDF → USD with articulation structure, physics properties, and collision geometry
2. **PhysX Drives** (position/velocity/force) provide more control than URDF `<transmission>` tags
3. **Physics Materials** define friction and restitution for realistic contact behavior
4. **Validation** ensures mass, inertia, collision, and joint limits are correct before simulation
5. **Batch conversion** automates importing multiple robots for large-scale experiments

---

## Further Reading

- [Isaac Sim URDF Importer Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/features/environment_setup/ext_omni_isaac_urdf.html)
- [PhysX ArticulationAPI Reference](https://docs.omniverse.nvidia.com/kit/docs/omni_usd_schema_physics/104.2/class_physx_schema_physx_articulation_a_p_i.html)
- [USD Physics Schema](https://openusd.org/release/api/usd_physics_page_front.html) - UsdPhys documentation

---

**Next**: [1.4 USD Scene Creation →](./1-4-scene-creation.md) - Build custom simulation environments with terrain, obstacles, and lighting!
