---
title: "1.4 USD Scene Creation"
description: "Build custom USD simulation environments with terrain, obstacles, PhysX configuration, RTX lighting, and PBR materials"
---

# 1.4 USD Scene Creation

## Introduction

Now that you can import robots to Isaac Sim (Section 1-3), it's time to create custom simulation environments. This section covers building USD scenes from scratch, including terrain, obstacles, physics configuration, and photorealistic lighting.

---

## USD Stage Construction

A **USD Stage** is the root container for your scene. It follows a hierarchical structure:

```
/World                     # Root prim (recommended naming)
├── /Environment           # Environment group
│   ├── /Ground           # Ground plane
│   └── /Obstacles        # Obstacles
├── /Robots               # Robots group
│   └── /Humanoid_01      # Your robot
└── /Lights               # Lighting
    ├── /DomeLight        # HDR skybox
    └── /AreaLight_01     # Studio lights
```

### Create New Stage

**Via GUI**:
1. **File** → **New** (or Ctrl+N)
2. Isaac Sim creates empty stage with default `/World` prim
3. Save as: **File** → **Save As** → `my_scene.usd`

**Via Python**:
```python
import omni.usd
from pxr import UsdGeom, Gf

# Get stage
stage = omni.usd.get_context().get_stage()

# Create World root prim
world_prim = stage.DefinePrim("/World", "Xform")
stage.SetDefaultPrim(world_prim)

# Save stage
stage.Export("/path/to/my_scene.usd")
```

---

## Adding Ground Plane

Every scene needs a ground plane for robots to stand on.

### Method 1: GUI Creation

1. **Create** → **Mesh** → **Plane**
2. Rename to `/World/Ground`
3. Set transform:
   - Position: (0, 0, 0)
   - Rotation: (0, 0, 0)
   - Scale: (10, 10, 1) - Creates 10m x 10m ground
4. Add physics:
   - Select `/World/Ground`
   - **Add** → **Physics** → **Collision Preset** → **Ground Plane**
   - This adds `CollisionAPI` and sets it as static (non-moving)

### Method 2: Python Creation

```python
from pxr import UsdGeom, UsdPhysics, PhysxSchema

# Create ground plane mesh
ground_path = "/World/Ground"
ground_geom = UsdGeom.Mesh.Define(stage, ground_path)

# Set geometry (100 vertices for 10m x 10m plane)
import numpy as np
size = 10.0
points = [
    (-size/2, -size/2, 0), (size/2, -size/2, 0),
    (size/2, size/2, 0), (-size/2, size/2, 0)
]
ground_geom.GetPointsAttr().Set(points)
ground_geom.GetFaceVertexIndicesAttr().Set([0, 1, 2, 3])
ground_geom.GetFaceVertexCountsAttr().Set([4])

# Add physics collision
collision_api = UsdPhysics.CollisionAPI.Apply(ground_geom.GetPrim())
collision_api.CreateCollisionEnabledAttr(True)

# Make it static (immovable)
UsdPhysics.RigidBodyAPI.Apply(ground_geom.GetPrim())
rigid_body = UsdPhysics.RigidBodyAPI(ground_geom.GetPrim())
rigid_body.CreateRigidBodyEnabledAttr(False)  # Static object
```

---

## Adding Obstacles

Place obstacles for navigation testing or collision experiments.

### Box Obstacles

```python
from pxr import UsdGeom, Gf

# Create box at position (2, 0, 0.5)
box_path = "/World/Obstacles/Box_01"
box_geom = UsdGeom.Cube.Define(stage, box_path)

# Set size and position
box_geom.GetSizeAttr().Set(1.0)  # 1m cube
xform = UsdGeom.Xformable(box_geom)
xform.AddTranslateOp().Set(Gf.Vec3d(2, 0, 0.5))  # Position

# Add physics
UsdPhysics.CollisionAPI.Apply(box_geom.GetPrim())
UsdPhysics.RigidBodyAPI.Apply(box_geom.GetPrim())
```

### Cylinder Obstacles

```python
# Create cylinder
cylinder_path = "/World/Obstacles/Cylinder_01"
cylinder_geom = UsdGeom.Cylinder.Define(stage, cylinder_path)

# Set dimensions
cylinder_geom.GetRadiusAttr().Set(0.3)  # 0.3m radius
cylinder_geom.GetHeightAttr().Set(2.0)  # 2m height
cylinder_geom.GetAxisAttr().Set("Z")    # Vertical orientation

# Position at (-2, 1, 1)
xform = UsdGeom.Xformable(cylinder_geom)
xform.AddTranslateOp().Set(Gf.Vec3d(-2, 1, 1))

# Add physics
UsdPhysics.CollisionAPI.Apply(cylinder_geom.GetPrim())
UsdPhysics.RigidBodyAPI.Apply(cylinder_geom.GetPrim())
```

---

## Using Omniverse Asset Store

Isaac Sim provides access to thousands of pre-made assets.

### Browse Assets

1. In Isaac Sim, open **Content** browser (bottom panel)
2. Navigate to **Omniverse://localhost/NVIDIA/Assets/**
3. Browse categories:
   - **Environments**: Warehouses, outdoor scenes
   - **Props**: Furniture, equipment
   - **Materials**: PBR material library

### Import Asset

**Drag and Drop**:
- Drag asset from Content browser → Viewport
- Asset appears at cursor position

**Reference Asset** (Recommended):
```python
from pxr import Usd

# Reference external USD file (doesn't duplicate, just links)
asset_prim = stage.DefinePrim("/World/Environment/Table", "Xform")
asset_prim.GetReferences().AddReference(
    "omniverse://localhost/NVIDIA/Assets/ArchVis/Furniture/table_01.usd"
)
```

Benefits: Small USD file size, assets update if source changes.

---

## PhysX Scene Configuration

Configure global physics parameters.

### Access Physics Settings

**GUI**: **Window** → **Physics** → **Settings**

### Key Parameters

| Parameter | Default | Humanoid Recommendation | Description |
|-----------|---------|------------------------|-------------|
| **Gravity** | (0, 0, -9.81) | (0, 0, -9.81) | Gravity vector (m/s²) |
| **Simulation Frequency** | 60 Hz | 60-120 Hz | Physics steps per second |
| **Position Iterations** | 4 | 16-32 | Solver iterations (higher = more stable) |
| **Velocity Iterations** | 1 | 1-2 | Velocity solver iterations |
| **Enable GPU Dynamics** | True | True | Use GPU for physics (faster) |
| **Broadphase Type** | GPU | GPU | Collision detection algorithm |

### Configure via Python

```python
from pxr import PhysxSchema, UsdPhysics

# Get or create physics scene
physics_scene_path = "/World/PhysicsScene"
physics_scene = UsdPhysics.Scene.Define(stage, physics_scene_path)

# Set gravity
physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
physics_scene.CreateGravityMagnitudeAttr().Set(9.81)

# Get PhysX-specific settings
physx_scene_api = PhysxSchema.PhysxSceneAPI.Apply(physics_scene.GetPrim())

# Configure solver
physx_scene_api.CreateEnableCCDAttr(True)  # Continuous Collision Detection
physx_scene_api.CreateEnableStabilizationAttr(True)
physx_scene_api.CreateSolverTypeAttr("TGS")  # Temporal Gauss-Seidel solver

# Set time step (1/60 = 60 Hz)
physx_scene_api.CreateTimeStepsPerSecondAttr(60.0)
```

### Collision Layers

Organize objects into collision groups:

```python
# Define collision groups (bitmask)
GROUND_GROUP = 0b0001    # Bit 0
ROBOT_GROUP = 0b0010     # Bit 1
OBSTACLE_GROUP = 0b0100  # Bit 2

# Set ground collision group
ground_prim = stage.GetPrimAtPath("/World/Ground")
physx_collision_api = PhysxSchema.PhysxCollisionAPI.Apply(ground_prim)
physx_collision_api.CreateCollisionGroupAttr(GROUND_GROUP)

# Robot collides with ground and obstacles, not other robots
robot_prim = stage.GetPrimAtPath("/World/Humanoid")
physx_collision_api = PhysxSchema.PhysxCollisionAPI.Apply(robot_prim)
physx_collision_api.CreateCollisionGroupAttr(ROBOT_GROUP)
physx_collision_api.CreateFilterGroupsAttr([GROUND_GROUP, OBSTACLE_GROUP])
```

---

## RTX Lighting Setup

Photorealistic rendering requires proper lighting.

### HDR Dome Light (Sky Environment)

**GUI**:
1. **Create** → **Light** → **Dome Light**
2. Select `/DomeLight`
3. In Properties → **Texture** → Load HDR image (e.g., `sunflower_puresky_4k.exr` from Asset Store)

**Python**:
```python
from pxr import UsdLux

# Create dome light
dome_light_path = "/World/Lights/DomeLight"
dome_light = UsdLux.DomeLight.Define(stage, dome_light_path)

# Load HDR texture
dome_light.CreateTextureFileAttr("omniverse://localhost/NVIDIA/Assets/Skies/Clear/sunflower_puresky_4k.exr")
dome_light.CreateIntensityAttr(1000.0)  # Brightness
dome_light.CreateTextureFormatAttr("latlong")  # Spherical mapping
```

### Area Lights (Studio Lighting)

```python
# Create rectangular area light
area_light_path = "/World/Lights/AreaLight_01"
area_light = UsdLux.RectLight.Define(stage, area_light_path)

# Configure light
area_light.CreateIntensityAttr(5000.0)  # Brightness (5000-50000 typical)
area_light.CreateWidthAttr(2.0)         # 2m wide
area_light.CreateHeightAttr(2.0)        # 2m tall
area_light.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))  # White light

# Position above and in front of robot
xform = UsdGeom.Xformable(area_light)
xform.AddTranslateOp().Set(Gf.Vec3d(3, 0, 3))  # 3m forward, 3m up
xform.AddRotateXYZOp().Set(Gf.Vec3d(-45, 0, 0))  # Point downward 45°
```

### Three-Point Lighting Setup

Professional studio lighting:

```python
# Key light (main, brightest)
key_light = UsdLux.RectLight.Define(stage, "/World/Lights/KeyLight")
key_light.CreateIntensityAttr(10000.0)
# Position: front-right, 45° above

# Fill light (softer, fills shadows)
fill_light = UsdLux.RectLight.Define(stage, "/World/Lights/FillLight")
fill_light.CreateIntensityAttr(5000.0)
# Position: front-left, same height as key

# Back light (rim light, separates subject from background)
back_light = UsdLux.RectLight.Define(stage, "/World/Lights/BackLight")
back_light.CreateIntensityAttr(7000.0)
# Position: behind robot, slightly above
```

---

## PBR Materials

Physically-Based Rendering materials add realism.

### Apply Material to Ground

**GUI**:
1. Select `/World/Ground`
2. **Create** → **Material** → **OmniPBR**
3. In Material properties:
   - **Diffuse Color**: (0.5, 0.5, 0.5) - Gray
   - **Roughness**: 0.8 - Matte surface
   - **Metallic**: 0.0 - Non-metallic

**Python**:
```python
from pxr import UsdShade

# Create material
material_path = "/World/Materials/GroundMaterial"
material = UsdShade.Material.Define(stage, material_path)

# Create OmniPBR shader
shader = UsdShade.Shader.Define(stage, f"{material_path}/Shader")
shader.CreateIdAttr("OmniPBR")

# Set shader inputs
shader.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.5, 0.5, 0.5))
shader.CreateInput("roughness_constant", Sdf.ValueTypeNames.Float).Set(0.8)
shader.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(0.0)

# Connect shader to material outputs
material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "out")

# Bind material to ground
ground_prim = stage.GetPrimAtPath("/World/Ground")
UsdShade.MaterialBindingAPI(ground_prim).Bind(material)
```

### Material Presets

Common material types:

| Material | Roughness | Metallic | Diffuse Color | Use Case |
|----------|-----------|----------|---------------|----------|
| **Matte Plastic** | 0.8 | 0.0 | Any | Robot joints, housing |
| **Glossy Metal** | 0.2 | 1.0 | Silver/Gold | Robot body, structural parts |
| **Rubber** | 0.9 | 0.0 | Black | Robot feet for traction |
| **Glass** | 0.0 | 0.0 | Clear + refraction | Transparent parts |
| **Concrete** | 0.9 | 0.0 | Gray | Ground/walls |

---

## Camera Setup

Add cameras for different viewpoints.

```python
from pxr import UsdGeom

# Create camera
camera_path = "/World/Cameras/MainCamera"
camera = UsdGeom.Camera.Define(stage, camera_path)

# Set camera properties
camera.CreateFocalLengthAttr(24.0)  # Wide angle (24mm lens)
camera.CreateFocusDistanceAttr(5.0)  # 5m focus distance
camera.CreateFStopAttr(4.0)          # f/4.0 aperture (depth of field)

# Position camera
xform = UsdGeom.Xformable(camera)
xform.AddTranslateOp().Set(Gf.Vec3d(5, 0, 2))  # 5m away, 2m high
xform.AddRotateXYZOp().Set(Gf.Vec3d(-15, -90, 0))  # Look at origin

# Set as active viewport camera
from omni.kit.viewport.utility import get_active_viewport
viewport = get_active_viewport()
viewport.camera_path = camera_path
```

---

## Complete Scene Example

Putting it all together:

```python
import omni.usd
from pxr import UsdGeom, UsdLux, UsdPhysics, PhysxSchema, Gf

stage = omni.usd.get_context().get_stage()

# 1. Create ground
ground = UsdGeom.Mesh.Define(stage, "/World/Ground")
# ... (see ground plane section)

# 2. Load humanoid robot (from Section 1-3)
robot_prim = stage.DefinePrim("/World/Humanoid", "Xform")
robot_prim.GetReferences().AddReference("/path/to/humanoid.usd")

# 3. Add obstacles
box = UsdGeom.Cube.Define(stage, "/World/Obstacles/Box_01")
# ... (position and physics)

# 4. Configure PhysX
physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
physics_scene.CreateGravityMagnitudeAttr(9.81)
physx_api = PhysxSchema.PhysxSceneAPI.Apply(physics_scene.GetPrim())
physx_api.CreateTimeStepsPerSecondAttr(60.0)

# 5. Add lighting
dome_light = UsdLux.DomeLight.Define(stage, "/World/Lights/DomeLight")
dome_light.CreateIntensityAttr(1000.0)

# 6. Save scene
stage.Export("/path/to/humanoid_environment.usd")
print("Scene created successfully!")
```

---

## Key Takeaways

1. **USD stages** organize scenes hierarchically (/World/Environment/Robots/Lights)
2. **Ground planes** need collision properties to support robots
3. **Obstacles** can be primitives (cube, cylinder) or imported assets
4. **PhysX configuration** requires tuning solver iterations (16-32 for humanoids)
5. **HDR dome lights** provide realistic environment lighting
6. **PBR materials** add photorealism (roughness, metallic, color)
7. **Cameras** can be positioned programmatically for specific viewpoints

---

## Further Reading

- [USD Geom Primitives](https://openusd.org/release/api/usd_geom_page_front.html) - Creating meshes and shapes
- [PhysX Scene Configuration](https://docs.omniverse.nvidia.com/extensions/latest/ext_physics.html) - Physics parameters
- [RTX Lighting in Omniverse](https://docs.omniverse.nvidia.com/materials-and-rendering/latest/lighting.html) - Advanced lighting techniques

---

**Next**: [1.5 ROS 2 Integration →](./1-5-ros2-integration.md) - Connect Isaac Sim to ROS 2 for realistic robot testing!
