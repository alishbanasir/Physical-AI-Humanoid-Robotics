# Example 01: Basic World Creation

## Description

This example demonstrates how to create a basic Gazebo world with optimized physics configuration for humanoid robot simulation. You'll learn to configure the ODE physics engine, set gravity, adjust solver parameters, and achieve Real-Time Factor (RTF) ≥ 0.9.

## Learning Objectives

- Understand SDF (Simulation Description Format) world file structure
- Configure ODE physics engine parameters
- Set up lighting and scene properties
- Measure Real-Time Factor (RTF) performance

## Prerequisites

- ROS 2 Humble installed
- Gazebo Classic 11 installed (`ros-humble-gazebo-ros-pkgs`)
- Basic understanding of XML file format

## File Structure

```
example-01-world-creation/
├── worlds/
│   └── empty_world.world      # SDF world file with physics configuration
├── launch/
│   └── world.launch.py        # ROS 2 launch file
└── README.md                  # This file
```

## Usage

### Step 1: Navigate to ROS 2 Workspace

```bash
cd ~/hackathon1-humanoid-robotics-book/examples/ros2_ws
```

### Step 2: Source ROS 2 Environment

```bash
source /opt/ros/humble/setup.bash
```

### Step 3: Launch Gazebo with Empty World

```bash
# Option A: Direct world file launch
gazebo ../module-2/chapter-1/example-01-world-creation/worlds/empty_world.world

# Option B: Using ROS 2 launch file (recommended)
cd ~/hackathon1-humanoid-robotics-book/examples/module-2/chapter-1/example-01-world-creation
ros2 launch launch/world.launch.py
```

### Step 4: Verify Performance

1. **Check Gazebo GUI opens successfully**
   - You should see an empty world with a ground plane
   - Grid lines visible for spatial reference

2. **Measure Real-Time Factor (RTF)**
   - Look at bottom-right corner of Gazebo window
   - **Expected:** RTF ≥ 0.9 (simulation runs at ≥90% real-time speed)
   - **Good performance:** RTF = 1.0 (real-time)
   - **Excellent performance:** RTF > 1.0 (faster than real-time)

3. **Test Physics**
   - In Gazebo GUI, go to **Insert** tab
   - Add a **Box** model
   - Drag to position above ground (e.g., height = 2m)
   - Release and observe box falls due to gravity
   - Box should settle on ground without bouncing excessively

## Physics Parameters Explained

From `empty_world.world`:

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `<max_step_size>` | 0.001 | Physics timestep (1ms) - smaller = more accurate |
| `<real_time_update_rate>` | 1000 | Update frequency (1000 Hz) |
| `<gravity>` | 0 0 -9.81 | Earth gravity in m/s² |
| `<iters>` | 50 | Solver iterations - balance between accuracy and speed |
| `<cfm>` | 0.0 | Constraint Force Mixing - 0 = rigid contacts |
| `<erp>` | 0.2 | Error Reduction Parameter - how fast to correct errors |

## Expected Output

**Gazebo GUI:**
- Empty world with gray background
- Ground plane with grid lines
- Directional lighting casting shadows
- Camera positioned for good viewing angle (5m away, 2m height)

**Terminal Output:**
```
[gzserver-1] [INFO] [gazebo]: Gazebo server started
[gzclient-2] [INFO] [gazebo]: Gazebo client GUI started
[INFO] [launch]: All processes started
```

**Performance Metrics:**
- **RTF:** ≥ 0.9 (check Gazebo GUI bottom-right)
- **FPS:** ≥ 30 (client rendering frame rate)
- **CPU Usage:** <50% on reference hardware (i5-8th gen, 4 cores)

## Troubleshooting

### Issue: RTF < 0.9 (Simulation Too Slow)

**Symptoms:** Real-Time Factor displayed in Gazebo GUI is less than 0.9

**Solutions:**
1. **Reduce update rate:**
   ```xml
   <real_time_update_rate>500</real_time_update_rate>
   ```

2. **Disable shadows:**
   ```xml
   <shadows>false</shadows>
   ```

3. **Run headless (server only):**
   ```bash
   gzserver empty_world.world
   ```

### Issue: Gazebo GUI Doesn't Open

**Symptoms:** Terminal shows errors, no GUI window appears

**Solutions:**
1. **Check Gazebo installation:**
   ```bash
   gazebo --version
   # Should output: Gazebo multi-robot simulator, version 11.14.0
   ```

2. **Verify ROS 2 Gazebo packages:**
   ```bash
   ros2 pkg list | grep gazebo_ros
   # Should list: gazebo_ros, gazebo_plugins, etc.
   ```

3. **Check graphics drivers (NVIDIA):**
   ```bash
   nvidia-smi
   # Verify GPU is detected
   ```

### Issue: Box Falls Through Ground

**Symptoms:** Inserted box penetrates ground plane and disappears

**Solutions:**
1. **Increase contact stiffness** - Edit world file:
   ```xml
   <contact_surface_layer>0.0001</contact_surface_layer>
   ```

2. **Reduce timestep** for better collision detection:
   ```xml
   <max_step_size>0.0005</max_step_size>
   ```

## Next Steps

- **Example 02:** Learn to spawn the humanoid robot from URDF
- **Experiment:** Try modifying gravity (e.g., Moon: `-1.62`, Mars: `-3.71`)
- **Advanced:** Add obstacles (boxes, cylinders) to create a test environment

## Additional Resources

- [Gazebo SDF Specification](http://sdformat.org/spec?ver=1.7)
- [ODE Physics Engine Documentation](http://www.ode.org/ode-latest-userguide.html)
- [Gazebo Classic Tutorials](https://classic.gazebosim.org/tutorials)
