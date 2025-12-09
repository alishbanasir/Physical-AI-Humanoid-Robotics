# Example 01: Hello Isaac Sim

## Overview

This example demonstrates the most basic Isaac Sim workflow: creating a simple scene with a cube and ground plane, then running a physics simulation. The cube falls under gravity and lands on the ground, demonstrating PhysX 5 physics integration.

## Learning Objectives

- Initialize Isaac Sim `World` for simulation management
- Create basic USD primitives using `omni.isaac.core.objects`
- Configure physics timestep (60 Hz default)
- Run simulation loop with `world.step()`
- Measure Real-Time Factor (RTF) for performance evaluation
- Query object state during simulation (`get_world_pose()`)

## Prerequisites

- Isaac Sim 2023.1.1 installed (see Module 3, Chapter 1, Section 1-2)
- Python 3.10 (bundled with Isaac Sim)

## Usage

### Method 1: GUI Mode (With Viewport)

```bash
cd ~/.local/share/ov/pkg/isaac_sim-2023.1.1
./python.sh /path/to/examples/module-3/chapter-1-isaac-sim/01-hello-isaac-sim/main.py
```

**Expected Result**: Isaac Sim window opens, showing a blue cube falling onto a gray ground plane.

### Method 2: Headless Mode (No GUI, Faster)

```bash
cd ~/.local/share/ov/pkg/isaac_sim-2023.1.1
./isaac-sim.sh --headless --allow-root /path/to/examples/module-3/chapter-1-isaac-sim/01-hello-isaac-sim/main.py
```

**Expected Result**: Terminal output only, with significantly higher RTF (5-10x).

## Expected Output

```
============================================================
Hello Isaac Sim - Basic Physics Simulation
============================================================

[1/5] Creating Isaac Sim World...
✓ World created successfully

[2/5] Adding ground plane...
✓ Ground plane added at /World/Ground

[3/5] Adding dynamic cube...
✓ Cube added at /World/Cube (2m above ground)

[4/5] Initializing simulation...
✓ Simulation initialized

[5/5] Running simulation for 5 seconds...
------------------------------------------------------------
[t=0.00s] Cube Z-position: 2.000 m
[t=0.50s] Cube Z-position: 1.787 m
[t=1.00s] Cube Z-position: 1.099 m
[t=1.50s] Cube Z-position: 0.425 m
[t=2.00s] Cube Z-position: 0.251 m
[t=2.50s] Cube Z-position: 0.250 m
[t=3.00s] Cube Z-position: 0.250 m
[t=3.50s] Cube Z-position: 0.250 m
[t=4.00s] Cube Z-position: 0.250 m
[t=4.50s] Cube Z-position: 0.250 m
------------------------------------------------------------

✓ Simulation complete!

Performance Metrics:
  Simulation time: 5.00s
  Wall-clock time: 2.34s
  Real-Time Factor (RTF): 2.14x
  ✓ Real-time achieved (RTF ≥ 1.0)

Final cube position: [0.000, 0.000, 0.250] m
✓ Cube successfully fell to ground under gravity

============================================================
Hello Isaac Sim example completed successfully!
============================================================
```

## Key Concepts

### 1. World Class

The `World` class manages simulation state:
- **Physics timestep** (`physics_dt`): Time between physics updates (default: 1/60s = 60 Hz)
- **Rendering timestep** (`rendering_dt`): Time between frame renders (default: 1/60s = 60 Hz)
- **Stage units**: USD units to meters conversion (default: 1.0 → 1 unit = 1 meter)

### 2. Scene Management

Objects are added to `world.scene` using `world.scene.add()`:
- **GroundPlane**: Static collision surface (infinite mass, no movement)
- **DynamicCuboid**: Rigid body with mass (responds to gravity and collisions)

### 3. Simulation Loop

```python
world.reset()  # Initialize physics (call once before simulation)

for i in range(steps):
    world.step(render=True)  # Step physics + render viewport
```

**render=True**: Updates viewport (slower, ~1-2x RTF)
**render=False**: Headless simulation (faster, ~5-10x RTF)

### 4. Real-Time Factor (RTF)

RTF measures simulation speed relative to real-time:
- **RTF = 1.0**: Simulation runs at real-time speed (1 sim second = 1 wall-clock second)
- **RTF > 1.0**: Faster than real-time (e.g., RTF=2.0 → 2 sim seconds per 1 wall-clock second)
- **RTF < 1.0**: Slower than real-time (complex scenes, insufficient GPU)

**Target**: RTF ≥ 1.0 for real-time robotics control.

## Troubleshooting

### Issue: RTF < 1.0 (Slow Simulation)

**Causes**:
- Insufficient GPU (NVIDIA RTX 2060 or lower)
- Rendering overhead (viewport updates)

**Solutions**:
1. Run headless: `./isaac-sim.sh --headless`
2. Reduce rendering frequency: Set `rendering_dt=1.0/30.0` (30 Hz instead of 60 Hz)
3. Use cloud GPU (AWS EC2 G4/G5 instances)

### Issue: Cube Doesn't Fall

**Cause**: Physics not initialized or gravity disabled.

**Solution**:
1. Ensure `world.reset()` is called before `world.step()`
2. Verify gravity in Physics Settings: **Window → Physics → Settings** → Gravity = (0, 0, -9.81)

### Issue: "Module not found: omni.isaac.core"

**Cause**: Using system Python instead of Isaac Sim Python.

**Solution**: Always use `./python.sh` or `./isaac-sim.sh` from Isaac Sim installation directory.

## Modifications

### Experiment 1: Change Cube Size

```python
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube",
        name="test_cube",
        position=np.array([0.0, 0.0, 5.0]),  # Start higher
        size=1.0,  # Larger cube (1m instead of 0.5m)
        color=np.array([1.0, 0.0, 0.0])  # Red cube
    )
)
```

### Experiment 2: Add Multiple Cubes

```python
for i in range(5):
    cube = world.scene.add(
        DynamicCuboid(
            prim_path=f"/World/Cube_{i}",
            name=f"cube_{i}",
            position=np.array([i * 0.6, 0.0, 2.0]),
            size=0.5,
            color=np.array([i/5.0, 0.5, 1.0 - i/5.0])
        )
    )
```

### Experiment 3: Measure Impact Velocity

```python
# Before final position check:
velocity, _ = cube.get_linear_velocity()
speed = np.linalg.norm(velocity)
print(f"Impact velocity: {speed:.3f} m/s")
```

## Next Steps

- **Example 02**: Import URDF robots to Isaac Sim
- **Example 03**: Batch URDF-to-USD conversion
- **Example 04**: Create custom scenes with obstacles
- **Module 3, Chapter 1, Section 1-6**: Learn Python automation API in detail

## Related Documentation

- [Isaac Core API: World](https://docs.omniverse.nvidia.com/isaacsim/latest/core_api_tutorials/tutorial_core_hello_world.html)
- [PhysX 5 Documentation](https://nvidia-omniverse.github.io/PhysX/physx/5.3.1/index.html)
- [USD Primitives](https://openusd.org/release/api/usd_geom_page_front.html)
