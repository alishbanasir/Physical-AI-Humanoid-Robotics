---
title: "1.6 Python Automation"
description: "Automate Isaac Sim workflows using omni.isaac.core API for batch testing, data collection, and reproducible experiments"
---

# 1.6 Python Automation

## Introduction

Isaac Sim's **Python API** (`omni.isaac.core`) enables complete programmatic control of simulation workflows. This allows you to:
- Automate repetitive tasks (spawn robots, configure scenes, run experiments)
- Collect large-scale datasets for machine learning (thousands of simulation runs)
- Run batch tests with parameter sweeps (physics configurations, controller gains)
- Create reproducible experiments with version-controlled scripts
- Integrate Isaac Sim into CI/CD pipelines for automated testing

This section teaches the core `omni.isaac.core` API patterns and demonstrates practical automation examples.

---

## Prerequisites

Before starting, ensure:
1. **Isaac Sim 2023.1.1** installed (Section 1-2)
2. **Python 3.10** (bundled with Isaac Sim)
3. Familiarity with USD concepts (Sections 1-3, 1-4)
4. Your Module 1 humanoid robot in USD format (Section 1-3)

---

## Isaac Sim Python Environment

Isaac Sim uses a bundled Python interpreter with custom packages.

### Method 1: Isaac Sim Python Shell

```bash
# Navigate to Isaac Sim installation
cd ~/.local/share/ov/pkg/isaac_sim-2023.1.1

# Launch Python shell with Isaac Sim packages
./python.sh

# Interactive Python prompt
>>> import omni.isaac.core
>>> print("Isaac Core API loaded")
```

### Method 2: Standalone Script Execution

```bash
# Run Python script with Isaac Sim interpreter
./python.sh /path/to/your_script.py

# Or make script executable
chmod +x /path/to/your_script.py
./python.sh /path/to/your_script.py
```

### Method 3: Headless Execution (No GUI)

For batch processing and CI/CD:

```bash
# Run headless (no viewport rendering, maximizes RTF)
./isaac-sim.sh --headless --allow-root /path/to/your_script.py

# Or via python.sh with headless flag
./python.sh /path/to/your_script.py --headless
```

**Benefits**: RTF 2.0-10.0 in headless mode (vs 0.5-2.0 with GUI rendering).

---

## Core API: World and Scene Management

### Creating a World

The `World` class manages simulation state and timestep:

```python
from omni.isaac.core import World

# Create world with physics parameters
world = World(
    stage_units_in_meters=1.0,  # USD units (1 unit = 1 meter)
    physics_dt=1.0/60.0,         # Physics timestep (60 Hz)
    rendering_dt=1.0/60.0        # Rendering timestep (60 Hz)
)

# Reset simulation (initializes physics)
world.reset()

print(f"World created, current time: {world.current_time:.2f}s")
```

### Simulation Loop

```python
# Run simulation for 10 seconds (600 steps at 60 Hz)
for i in range(600):
    world.step(render=True)  # Step physics and render

    # Get current simulation time
    current_time = world.current_time

    if i % 60 == 0:  # Print every 1 second
        print(f"Step {i}, Time: {current_time:.2f}s")

# Stop simulation
world.stop()
```

**Key Methods**:
- `world.reset()`: Initialize simulation (call before first step)
- `world.step(render=True)`: Step physics and rendering
- `world.play()`: Start continuous simulation
- `world.pause()`: Pause simulation
- `world.stop()`: Stop simulation
- `world.current_time`: Get simulation time (seconds)

---

## Core API: Loading Robots

### Method 1: Add Robot from USD File

```python
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage

# Add robot USD to stage
robot_prim_path = "/World/humanoid"
robot_usd_path = "/path/to/humanoid.usd"

add_reference_to_stage(usd_path=robot_usd_path, prim_path=robot_prim_path)

# Create Robot object
robot = world.scene.add(
    Robot(
        prim_path=robot_prim_path,
        name="humanoid_01"
    )
)

print(f"Robot loaded: {robot.name} at {robot_prim_path}")
```

### Method 2: Add Articulation (Advanced Control)

For joint-level control:

```python
from omni.isaac.core.articulations import Articulation

# Create articulation
robot = world.scene.add(
    Articulation(
        prim_path="/World/humanoid",
        name="humanoid_01"
    )
)

# Initialize articulation (after world.reset())
world.reset()
robot.initialize()

# Get joint information
num_dof = robot.num_dof
joint_names = robot.dof_names
joint_positions = robot.get_joint_positions()
joint_velocities = robot.get_joint_velocities()

print(f"Robot has {num_dof} DOF: {joint_names}")
print(f"Current positions: {joint_positions}")
```

---

## Core API: Joint Control

### Position Control

Set target joint positions (PD control):

```python
# Define target positions (radians)
target_positions = [0.0] * robot.num_dof  # All joints to 0
target_positions[0] = 0.5   # First joint to 0.5 rad
target_positions[1] = -0.3  # Second joint to -0.3 rad

# Set joint position targets
robot.set_joint_position_targets(target_positions)

# Step simulation to apply control
for _ in range(100):
    world.step(render=True)

    # Check if reached target
    current_pos = robot.get_joint_positions()
    error = abs(current_pos[0] - target_positions[0])

    if error < 0.01:  # Within 0.01 rad
        print("Target reached!")
        break
```

### Velocity Control

Set target joint velocities:

```python
# Define target velocities (rad/s)
target_velocities = [0.0] * robot.num_dof
target_velocities[0] = 1.0   # First joint at 1 rad/s
target_velocities[1] = -0.5  # Second joint at -0.5 rad/s

# Set joint velocity targets
robot.set_joint_velocity_targets(target_velocities)

# Run for 2 seconds
for _ in range(120):  # 120 steps = 2 seconds at 60 Hz
    world.step(render=True)
```

### Effort (Torque) Control

Apply direct joint torques:

```python
# Define joint efforts (N·m)
efforts = [0.0] * robot.num_dof
efforts[0] = 10.0   # 10 N·m torque on first joint
efforts[1] = -5.0   # -5 N·m on second joint

# Apply efforts
robot.set_joint_efforts(efforts)

# Step simulation
for _ in range(100):
    world.step(render=True)
```

---

## Core API: Observations and State

### Get Robot State

```python
# Joint positions (radians)
positions = robot.get_joint_positions()

# Joint velocities (rad/s)
velocities = robot.get_joint_velocities()

# Joint efforts (N·m)
efforts = robot.get_applied_joint_efforts()

# World pose (position + orientation quaternion)
world_pose = robot.get_world_pose()
position, orientation = world_pose  # position: [x, y, z], orientation: [w, x, y, z]

# Link velocities (linear + angular)
linear_velocity = robot.get_linear_velocity()    # [vx, vy, vz]
angular_velocity = robot.get_angular_velocity()  # [wx, wy, wz]

print(f"Robot position: {position}")
print(f"Robot orientation (quat): {orientation}")
print(f"Joint 0 position: {positions[0]:.3f} rad")
print(f"Joint 0 velocity: {velocities[0]:.3f} rad/s")
```

### Compute Jacobians (Advanced)

For inverse kinematics and dynamics:

```python
# Get Jacobian matrix (6 x num_dof)
# Relates joint velocities to end-effector velocity
jacobian = robot.get_jacobian()

print(f"Jacobian shape: {jacobian.shape}")  # (6, num_dof)
```

---

## Practical Example 1: Parameter Sweep

Test humanoid stability across different PhysX solver iterations:

```python
import omni.isaac.core
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from pxr import PhysxSchema
import time
import csv

# Solver iteration configurations to test
solver_configs = [4, 8, 16, 24, 32]
results = []

for solver_iter in solver_configs:
    print(f"\n=== Testing solver iterations: {solver_iter} ===")

    # Create world
    world = World()

    # Load robot
    robot = world.scene.add(
        Articulation(
            prim_path="/World/humanoid",
            name="humanoid"
        )
    )

    # Configure PhysX solver
    stage = omni.usd.get_context().get_stage()
    physics_scene_path = "/World/PhysicsScene"
    physics_scene = stage.GetPrimAtPath(physics_scene_path)
    physx_api = PhysxSchema.PhysxSceneAPI.Apply(physics_scene)
    physx_api.CreateSolverPositionIterationCountAttr(solver_iter)

    # Reset simulation
    world.reset()
    robot.initialize()

    # Measure RTF and stability
    start_time = time.time()
    sim_duration = 10.0  # 10 seconds sim time
    steps = int(sim_duration / world.get_physics_dt())

    stable = True
    for i in range(steps):
        world.step(render=False)  # Headless for speed

        # Check stability (robot base Z > 0.5m)
        position, _ = robot.get_world_pose()
        if position[2] < 0.5:  # Fallen
            stable = False
            print(f"  Robot fell at step {i}")
            break

    wall_time = time.time() - start_time
    rtf = sim_duration / wall_time

    # Record results
    results.append({
        "solver_iterations": solver_iter,
        "rtf": rtf,
        "stable": stable
    })

    print(f"  RTF: {rtf:.2f}, Stable: {stable}")

    # Clean up
    world.clear()

# Save results to CSV
with open("solver_sweep_results.csv", "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=["solver_iterations", "rtf", "stable"])
    writer.writeheader()
    writer.writerows(results)

print("\nResults saved to solver_sweep_results.csv")
```

**Expected Output**:
```
=== Testing solver iterations: 4 ===
  Robot fell at step 23
  RTF: 5.23, Stable: False

=== Testing solver iterations: 8 ===
  Robot fell at step 156
  RTF: 4.87, Stable: False

=== Testing solver iterations: 16 ===
  RTF: 3.54, Stable: True

...
```

---

## Practical Example 2: Multi-Robot Spawning

Spawn 5 humanoid robots in array formation for parallel testing:

```python
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import add_reference_to_stage
import numpy as np

# Create world
world = World()

# Robot USD path
robot_usd_path = "/path/to/humanoid.usd"

# Spawn 5 robots in 2m spacing
robots = []
for i in range(5):
    # Define spawn position
    x_position = i * 2.0  # 0, 2, 4, 6, 8 meters
    spawn_position = np.array([x_position, 0.0, 1.0])  # 1m above ground

    # Add robot to stage
    robot_prim_path = f"/World/humanoid_{i:02d}"
    add_reference_to_stage(usd_path=robot_usd_path, prim_path=robot_prim_path)

    # Create articulation
    robot = world.scene.add(
        Articulation(
            prim_path=robot_prim_path,
            name=f"humanoid_{i:02d}",
            position=spawn_position
        )
    )

    robots.append(robot)
    print(f"Spawned {robot.name} at {spawn_position}")

# Initialize simulation
world.reset()

# Control all robots independently
for i in range(600):  # 10 seconds
    world.step(render=True)

    # Apply different control to each robot
    for idx, robot in enumerate(robots):
        # Example: Oscillate first joint
        target_pos = [0.0] * robot.num_dof
        target_pos[0] = 0.5 * np.sin(2 * np.pi * 0.5 * world.current_time + idx)
        robot.set_joint_position_targets(target_pos)

print("Multi-robot simulation complete")
```

---

## Practical Example 3: Data Collection

Collect joint trajectory data for machine learning:

```python
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
import numpy as np
import json

# Create world and robot
world = World()
robot = world.scene.add(
    Articulation(prim_path="/World/humanoid", name="humanoid")
)

world.reset()
robot.initialize()

# Data collection
trajectory_data = []

for step in range(1000):  # Collect 1000 samples
    world.step(render=False)  # Headless for speed

    # Apply random joint commands (exploration)
    random_positions = np.random.uniform(
        low=-0.5, high=0.5, size=robot.num_dof
    )
    robot.set_joint_position_targets(random_positions.tolist())

    # Collect observations
    obs = {
        "step": step,
        "time": world.current_time,
        "joint_positions": robot.get_joint_positions().tolist(),
        "joint_velocities": robot.get_joint_velocities().tolist(),
        "joint_efforts": robot.get_applied_joint_efforts().tolist(),
        "base_position": robot.get_world_pose()[0].tolist(),
        "base_orientation": robot.get_world_pose()[1].tolist()
    }

    trajectory_data.append(obs)

    if step % 100 == 0:
        print(f"Collected {step} samples...")

# Save to JSON
with open("humanoid_trajectory.json", "w") as f:
    json.dump(trajectory_data, f, indent=2)

print(f"Saved {len(trajectory_data)} samples to humanoid_trajectory.json")
```

**Use Cases**:
- Supervised learning for inverse kinematics
- Imitation learning datasets
- System identification for model-based control

---

## Practical Example 4: Camera Data Export

Capture RGB images and depth maps programmatically:

```python
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.sensor import Camera
import numpy as np
from PIL import Image

# Create world
world = World()

# Add camera
camera = Camera(
    prim_path="/World/Camera",
    frequency=30,  # 30 Hz
    resolution=(640, 480)
)

# Position camera
camera.set_world_pose(
    position=np.array([3.0, 0.0, 1.5]),  # 3m in front, 1.5m high
    orientation=np.array([0.0, 0.0, 1.0, 0.0])  # Look at origin
)

world.reset()
camera.initialize()

# Capture images
for i in range(100):  # Capture 100 frames
    world.step(render=True)

    # Get camera data
    rgb_data = camera.get_rgba()[:, :, :3]  # Remove alpha channel
    depth_data = camera.get_depth()

    # Save every 10th frame
    if i % 10 == 0:
        # Save RGB
        rgb_image = Image.fromarray(rgb_data.astype(np.uint8))
        rgb_image.save(f"frame_{i:04d}_rgb.png")

        # Save depth (normalize to 0-255)
        depth_normalized = (depth_data / depth_data.max() * 255).astype(np.uint8)
        depth_image = Image.fromarray(depth_normalized)
        depth_image.save(f"frame_{i:04d}_depth.png")

        print(f"Saved frame {i}")

print("Image capture complete")
```

---

## Headless Execution Best Practices

For CI/CD and cloud deployment:

### 1. Disable GUI Rendering

```python
from omni.isaac.core import World

# Create headless world (no viewport)
world = World(
    stage_units_in_meters=1.0,
    physics_dt=1.0/60.0,
    rendering_dt=1.0/60.0
)

# Step without rendering
for i in range(1000):
    world.step(render=False)  # render=False for headless
```

### 2. Launch with Headless Flag

```bash
# Run script without GUI
./isaac-sim.sh --headless --allow-root /path/to/script.py
```

### 3. Measure Performance

```python
import time

start_time = time.time()
sim_duration = 100.0  # 100 seconds sim time

for i in range(6000):  # 100 seconds at 60 Hz
    world.step(render=False)

wall_time = time.time() - start_time
rtf = sim_duration / wall_time

print(f"Real-Time Factor: {rtf:.2f}x")
# Expected: RTF 5-10x in headless mode on RTX 4090
```

### 4. Error Handling

```python
try:
    # Simulation code
    world = World()
    world.reset()

    for i in range(1000):
        world.step(render=False)

except Exception as e:
    print(f"Simulation error: {e}")

finally:
    # Clean up
    world.clear()
    print("Cleanup complete")
```

---

## Integration with External Tools

### 1. NumPy and Scientific Python

```python
import numpy as np
from scipy.spatial.transform import Rotation

# Convert quaternion to Euler angles
position, quat = robot.get_world_pose()
rotation = Rotation.from_quat([quat[1], quat[2], quat[3], quat[0]])  # [x,y,z,w]
euler_angles = rotation.as_euler('xyz', degrees=True)

print(f"Roll: {euler_angles[0]:.1f}°, Pitch: {euler_angles[1]:.1f}°, Yaw: {euler_angles[2]:.1f}°")
```

### 2. PyTorch Integration (RL Training)

```python
import torch
import numpy as np

# Collect observations
obs = robot.get_joint_positions()
obs_tensor = torch.tensor(obs, dtype=torch.float32)

# Run policy network (example)
# action = policy_network(obs_tensor)

# Apply actions
# robot.set_joint_position_targets(action.numpy().tolist())
```

### 3. Logging and Visualization

```python
import matplotlib.pyplot as plt

# Collect data
times = []
positions = []

for i in range(600):
    world.step(render=False)

    times.append(world.current_time)
    pos = robot.get_joint_positions()[0]  # First joint
    positions.append(pos)

# Plot
plt.plot(times, positions)
plt.xlabel("Time (s)")
plt.ylabel("Joint 0 Position (rad)")
plt.title("Joint Trajectory")
plt.savefig("joint_trajectory.png")
print("Plot saved to joint_trajectory.png")
```

---

## Key Takeaways

1. **omni.isaac.core API** provides complete programmatic control over Isaac Sim simulations
2. **World class** manages simulation state, timestep, and scene
3. **Articulation class** enables joint-level control (position, velocity, effort)
4. **Headless execution** achieves RTF 5-10x for batch processing and CI/CD
5. **Python automation** enables reproducible experiments, data collection, and parameter sweeps
6. **Integration with NumPy, PyTorch, ROS 2** unlocks advanced workflows (RL training, perception, control)

---

## Further Reading

- [Isaac Core API Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/core_api_tutorials/index.html) - Official tutorials
- [Omniverse Python Scripting](https://docs.omniverse.nvidia.com/kit/docs/kit-manual/latest/guide/python_scripting.html) - Advanced scripting patterns
- [Isaac Sim Headless Mode](https://docs.omniverse.nvidia.com/isaacsim/latest/manual_standalone_python.html) - Deployment guide
- [USD Python API](https://openusd.org/release/api/index.html) - Low-level USD manipulation

---

**Next**: [1.7 Exercises and Summary →](./1-7-exercises.md) - Test your Chapter 1 knowledge with hands-on exercises!
