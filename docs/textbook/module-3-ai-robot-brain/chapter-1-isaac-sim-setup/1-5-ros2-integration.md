---
title: "1.5 ROS 2 Integration"
description: "Enable Isaac Sim ROS 2 bridge extensions to publish sensor data, subscribe to control commands, and synchronize simulation time"
---

# 1.5 ROS 2 Integration

## Introduction

Isaac Sim provides native **ROS 2 integration** via bridge extensions, enabling bidirectional communication between simulation and ROS 2 nodes. This allows you to:
- Publish sensor data (/joint_states, /camera/image_raw, /scan) from Isaac Sim to ROS 2
- Subscribe to control commands (/cmd_vel, /joint_commands) from ROS 2 to Isaac Sim
- Synchronize simulation time with /clock topic for deterministic testing
- Test ROS 2 navigation, perception, and control algorithms in high-fidelity simulation before real-world deployment

This section teaches you to configure the ROS 2 bridge, publish/subscribe topics, and validate communication using `ros2 topic` commands.

---

## Prerequisites

Before starting, ensure:
1. **ROS 2 Humble** installed on your system (see [ROS 2 Humble Installation](https://docs.ros.org/en/humble/Installation.html))
2. **Isaac Sim 2023.1.1** installed (Section 1-2)
3. Your Module 1 humanoid robot imported to USD format (Section 1-3)
4. Environment sourced:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

---

## Step 1: Enable ROS 2 Bridge Extension

### Method A: GUI Activation

1. Launch Isaac Sim
2. Go to **Window** → **Extensions**
3. Search for **"ROS2 Bridge"**
4. Toggle switch to **ON** (extension loads automatically)
5. Verify activation: Look for ROS 2 icon in Extensions panel

**Screenshot Placeholder**: `ros2-bridge-extension-enabled.png` - Extensions window showing ROS 2 Bridge active

### Method B: Python Activation

```python
from omni.isaac.core.utils.extensions import enable_extension

# Enable ROS 2 bridge
enable_extension("omni.isaac.ros2_bridge")

print("ROS 2 Bridge enabled")
```

---

## Step 2: Configure ROS 2 Domain ID (Optional)

If using multiple ROS 2 systems on the same network, set domain ID:

```bash
# In terminal before launching Isaac Sim:
export ROS_DOMAIN_ID=42  # Choose 0-101

# Launch Isaac Sim
./isaac-sim.sh
```

**Or set in Isaac Sim Python script:**
```python
import os
os.environ["ROS_DOMAIN_ID"] = "42"
```

Default is `ROS_DOMAIN_ID=0` (fine for single-machine testing).

---

## Step 3: Publish Joint States

Joint states publish current positions, velocities, and efforts for all robot joints.

### GUI Configuration

1. Load your humanoid robot in Isaac Sim (Section 1-3)
2. Select robot root prim in Stage panel (e.g., `/World/humanoid`)
3. Go to **Isaac Utils** → **ROS 2 Publishers** → **Joint State Publisher**
4. Configure:
   - **Topic Name**: `/joint_states` (standard ROS 2 topic)
   - **Publish Rate**: 50 Hz (recommended for humanoids)
   - **Include Positions**: ☑ Checked
   - **Include Velocities**: ☑ Checked
   - **Include Efforts**: ☑ Checked (torques)
5. Click **Enable**
6. Start simulation (▶️ Play)

### Python Configuration

```python
import omni.isaac.core.utils.stage as stage_utils
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.extensions import enable_extension

# Enable ROS 2 bridge
enable_extension("omni.isaac.ros2_bridge")

# Get articulation (humanoid robot)
robot_path = "/World/humanoid"
robot = Articulation(prim_path=robot_path)

# Import ROS 2 bridge API
import omni.isaac.ros2_bridge

# Create joint state publisher
joint_state_pub = omni.isaac.ros2_bridge.create_joint_state_publisher(
    prim_path=robot_path,
    topic_name="/joint_states",
    publish_rate=50.0  # Hz
)

print("Joint state publisher created on /joint_states at 50 Hz")
```

### Verify Joint States in ROS 2

Open new terminal and verify topic is publishing:

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# List topics (should see /joint_states)
ros2 topic list

# Echo joint states
ros2 topic echo /joint_states
```

**Expected Output**:
```yaml
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: 'base_link'
name:
  - left_hip_yaw_joint
  - left_hip_roll_joint
  - left_hip_pitch_joint
  # ... (all 30+ joints)
position:
  - 0.0523  # radians
  - -0.0234
  # ...
velocity:
  - 0.12   # rad/s
  - -0.05
  # ...
effort:
  - 15.3   # N·m
  - -8.2
  # ...
```

---

## Step 4: Publish Camera Images

Publish RGB camera images as ROS 2 `sensor_msgs/Image` messages.

### Add Camera to Scene

```python
from pxr import UsdGeom, Gf
from omni.isaac.core.utils.stage import add_reference_to_stage

# Create camera
camera_path = "/World/Camera"
camera = UsdGeom.Camera.Define(stage_utils.get_current_stage(), camera_path)

# Set camera properties
camera.CreateFocalLengthAttr(24.0)  # Wide angle
camera.CreateFocusDistanceAttr(3.0)  # 3m focus

# Position camera (3m in front of robot, 1.5m high)
xform = UsdGeom.Xformable(camera)
xform.AddTranslateOp().Set(Gf.Vec3d(3, 0, 1.5))
xform.AddRotateXYZOp().Set(Gf.Vec3d(0, -180, 0))  # Look at robot
```

### Publish Camera Topic

```python
import omni.isaac.ros2_bridge

# Create RGB camera publisher
camera_pub = omni.isaac.ros2_bridge.create_camera_publisher(
    camera_prim_path="/World/Camera",
    topic_name="/camera/image_raw",
    publish_rate=30.0  # Hz (30 FPS)
)

print("Camera publisher created on /camera/image_raw at 30 Hz")
```

### Verify Camera Images in ROS 2

```bash
# List image topics
ros2 topic list | grep image

# Echo image metadata (not pixels, too large)
ros2 topic echo /camera/image_raw --no-arr

# Visualize in RViz2
rviz2
# Add → By Topic → /camera/image_raw → Image
```

**Screenshot Placeholder**: `rviz-isaac-sim-camera.png` - RViz2 showing Isaac Sim camera feed

---

## Step 5: Publish LiDAR Scans (Optional)

For navigation testing, publish 2D LiDAR scans.

### Add LiDAR Sensor

```python
import omni.isaac.range_sensor

# Create 2D LiDAR sensor
lidar_path = "/World/humanoid/lidar"
lidar_config = {
    "min_range": 0.1,        # meters
    "max_range": 10.0,       # meters
    "horizontal_fov": 270.0, # degrees
    "horizontal_resolution": 1.0,  # degrees per ray (270 rays)
    "rotation_rate": 10.0,   # Hz
    "high_lod": True,        # High quality
    "yaw_offset": 0.0,
    "draw_points": True,     # Visualize rays
    "draw_lines": True
}

# Create LiDAR
lidar = omni.isaac.range_sensor.create_lidar(
    prim_path=lidar_path,
    config=lidar_config,
    position=Gf.Vec3d(0, 0, 0.5),  # 0.5m above base_link
    parent_prim_path="/World/humanoid/base_link"
)
```

### Publish LiDAR Topic

```python
import omni.isaac.ros2_bridge

# Create LaserScan publisher
lidar_pub = omni.isaac.ros2_bridge.create_lidar_publisher(
    lidar_prim_path="/World/humanoid/lidar",
    topic_name="/scan",
    publish_rate=10.0  # Hz
)

print("LiDAR publisher created on /scan at 10 Hz")
```

### Verify LiDAR in ROS 2

```bash
# Echo scan data
ros2 topic echo /scan

# Visualize in RViz2
rviz2
# Add → By Topic → /scan → LaserScan
# Set Fixed Frame to "base_link" or "lidar_frame"
```

---

## Step 6: Subscribe to Control Commands

Subscribe to ROS 2 topics to control the robot from external nodes.

### Method A: Velocity Commands (/cmd_vel)

For mobile base control (if your humanoid has wheels or can walk):

```python
import omni.isaac.ros2_bridge

# Create cmd_vel subscriber
cmd_vel_sub = omni.isaac.ros2_bridge.create_twist_subscriber(
    prim_path="/World/humanoid",
    topic_name="/cmd_vel"
)

# In simulation loop, apply velocities to robot
# (Note: Isaac Sim handles this automatically if ArticulationController is configured)

print("Subscribed to /cmd_vel for velocity commands")
```

**Test from ROS 2**:
```bash
# Publish twist command (1 m/s forward)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Method B: Joint Commands (/joint_commands)

For direct joint position/velocity/effort control:

```python
import omni.isaac.ros2_bridge

# Create joint command subscriber
joint_cmd_sub = omni.isaac.ros2_bridge.create_joint_command_subscriber(
    prim_path="/World/humanoid",
    topic_name="/joint_commands",
    command_type="position"  # Options: "position", "velocity", "effort"
)

print("Subscribed to /joint_commands for position control")
```

**Test from ROS 2**:
```bash
# Publish joint positions (example: 2 joints)
ros2 topic pub /joint_commands sensor_msgs/msg/JointState "{
  name: ['left_hip_yaw_joint', 'left_hip_roll_joint'],
  position: [0.5, -0.3]
}" --once
```

### Custom Subscriber (Python Callback)

For advanced control logic:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_commands',
            self.joint_command_callback,
            10
        )

    def joint_command_callback(self, msg):
        # Apply joint commands to Isaac Sim robot
        for i, joint_name in enumerate(msg.name):
            position = msg.position[i]
            # Set joint target via Isaac Sim API
            # robot.set_joint_position(joint_name, position)
            print(f"Received command: {joint_name} -> {position} rad")

# Initialize ROS 2
rclpy.init()
controller = HumanoidController()
rclpy.spin(controller)
```

---

## Step 7: Clock Synchronization

Synchronize ROS 2 nodes with Isaac Sim's simulation time (critical for deterministic testing).

### Enable Simulation Time in Isaac Sim

```python
import omni.isaac.ros2_bridge

# Enable /clock topic publishing
clock_pub = omni.isaac.ros2_bridge.create_clock_publisher(
    topic_name="/clock",
    publish_rate=100.0  # Hz (high rate for smooth time updates)
)

print("Clock publisher created on /clock at 100 Hz")
```

### Configure ROS 2 Nodes to Use Sim Time

**Method 1: Launch File Parameter**
```xml
<launch>
  <node pkg="your_package" exec="your_node" name="your_node">
    <param name="use_sim_time" value="true"/>
  </node>
</launch>
```

**Method 2: Command Line**
```bash
ros2 run your_package your_node --ros-args -p use_sim_time:=true
```

**Method 3: Python Node**
```python
class MyNode(Node):
    def __init__(self):
        super().__init__('my_node',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.get_parameter_or('use_sim_time', rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True))
```

### Verify Clock Synchronization

```bash
# Echo /clock topic
ros2 topic echo /clock

# Check node is using sim time
ros2 param get /your_node use_sim_time
# Should output: Boolean value is: True
```

---

## Step 8: Complete Integration Example

Full Python script integrating all ROS 2 features:

```python
import omni.isaac.core
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.extensions import enable_extension
import omni.isaac.ros2_bridge

# Enable ROS 2 bridge
enable_extension("omni.isaac.ros2_bridge")

# Create Isaac Sim world
world = World(stage_units_in_meters=1.0)

# Load humanoid robot
robot_path = "/World/humanoid"
robot = world.scene.add(Articulation(prim_path=robot_path))

# Configure ROS 2 publishers
joint_state_pub = omni.isaac.ros2_bridge.create_joint_state_publisher(
    prim_path=robot_path,
    topic_name="/joint_states",
    publish_rate=50.0
)

camera_pub = omni.isaac.ros2_bridge.create_camera_publisher(
    camera_prim_path="/World/Camera",
    topic_name="/camera/image_raw",
    publish_rate=30.0
)

clock_pub = omni.isaac.ros2_bridge.create_clock_publisher(
    topic_name="/clock",
    publish_rate=100.0
)

# Configure ROS 2 subscribers
joint_cmd_sub = omni.isaac.ros2_bridge.create_joint_command_subscriber(
    prim_path=robot_path,
    topic_name="/joint_commands",
    command_type="position"
)

# Reset simulation
world.reset()

# Run simulation loop
print("Running Isaac Sim with ROS 2 bridge...")
print("Topics published: /joint_states, /camera/image_raw, /clock")
print("Topics subscribed: /joint_commands")

for i in range(10000):  # 10000 steps
    world.step(render=True)

    # Optional: Add custom control logic here
    if i % 100 == 0:
        print(f"Simulation step {i}, time: {world.current_time:.2f}s")

print("Simulation complete")
```

**Run Script**:
```bash
# Terminal 1: Launch Isaac Sim with script
cd ~/.local/share/ov/pkg/isaac_sim-2023.1.1
./python.sh /path/to/ros2_integration_example.py

# Terminal 2: Verify topics
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic echo /joint_states
```

---

## Latency Optimization

For real-time performance, minimize communication latency:

### 1. Use Intra-Process Communication (If Possible)

When ROS 2 nodes run in same process as Isaac Sim:
```python
# Enable intra-process comms (zero-copy)
rclpy.init(args=['--ros-args', '--enable-intra-process-comms'])
```

### 2. Adjust QoS (Quality of Service) Profiles

For high-frequency topics, use BEST_EFFORT reliability:
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy

qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
publisher = node.create_publisher(JointState, '/joint_states', qos)
```

**Tradeoff**: BEST_EFFORT reduces latency but may drop packets (acceptable for sensor data).

### 3. Reduce Publish Rates

Balance fidelity vs performance:
- **Joint states**: 50-100 Hz (humanoid dynamics)
- **Camera**: 15-30 Hz (sufficient for vision)
- **LiDAR**: 10-20 Hz (navigation)
- **Clock**: 100-200 Hz (time sync)

### 4. Use DDS Tuning (CycloneDDS Recommended)

CycloneDDS typically has lower latency than FastRTPS:
```bash
# Install CycloneDDS
sudo apt install ros-humble-rmw-cyclonedds-cpp

# Set as ROS 2 middleware
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Launch Isaac Sim
./isaac-sim.sh
```

**Measured Latency** (from research.md):
- Joint state round-trip: 5-15 ms (50 Hz publish rate, CycloneDDS)
- Camera image latency: 30-50 ms (30 FPS, 640x480 resolution)

---

## Troubleshooting

### Issue: ROS 2 Topics Not Appearing

**Symptom**: `ros2 topic list` shows no Isaac Sim topics

**Solution**:
1. Verify ROS 2 bridge extension enabled (**Window → Extensions → ROS2 Bridge**)
2. Check simulation is running (Press ▶️ Play) - topics only publish during simulation
3. Verify ROS_DOMAIN_ID matches between Isaac Sim and terminal:
   ```bash
   echo $ROS_DOMAIN_ID  # Should match Isaac Sim domain
   ```
4. Check ROS 2 installation sourced:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

---

### Issue: Joint Commands Not Applied

**Symptom**: Publishing to `/joint_commands` has no effect on robot

**Solution**:
1. Verify subscriber created correctly (check topic name matches)
2. Ensure ArticulationController is configured for command type (position/velocity/effort)
3. Check joint names in command message match robot joint names:
   ```bash
   ros2 topic echo /joint_states --once  # Get exact joint names
   ```
4. Verify drive API configured with sufficient stiffness/damping (Section 1-3)

---

### Issue: Camera Images Black/Corrupted

**Symptom**: `/camera/image_raw` publishes but images are black or corrupted

**Solution**:
1. Verify camera has valid viewport rendering enabled
2. Check RTX rendering active (**Viewport → Rendering Mode → RTX**)
3. Ensure scene has lighting (Section 1-4)
4. Wait 2-3 seconds after simulation start for RTX renderer to initialize

---

## Key Takeaways

1. **ROS 2 Bridge extension** enables bidirectional communication between Isaac Sim and ROS 2 ecosystem
2. **Joint states, camera images, and LiDAR scans** can be published as standard ROS 2 topics
3. **Control commands** (velocity, joint positions) can be subscribed from ROS 2 nodes
4. **Clock synchronization** with `/clock` topic ensures deterministic testing (set `use_sim_time=true`)
5. **Latency optimization** (CycloneDDS, BEST_EFFORT QoS, reduced publish rates) achieves 5-15 ms round-trip for joint commands
6. **Integration testing** in simulation validates perception, navigation, and control algorithms before real-world deployment

---

## Further Reading

- [Isaac Sim ROS 2 Bridge Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/index.html) - Official tutorials
- [ROS 2 QoS Policies](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html) - Understanding RELIABLE vs BEST_EFFORT
- [CycloneDDS Tuning Guide](https://github.com/ros2/rmw_cyclonedds) - DDS middleware optimization
- [RViz2 User Guide](https://github.com/ros2/rviz) - Visualizing Isaac Sim sensor data

---

**Next**: [1.6 Python Automation →](./1-6-python-scripting.md) - Automate simulation workflows with `omni.isaac.core` Python API!
