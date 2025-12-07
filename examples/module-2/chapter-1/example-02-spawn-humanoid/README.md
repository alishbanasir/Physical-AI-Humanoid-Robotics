# Example 02: Spawn Humanoid Robot

## Description
Demonstrates spawning the simple humanoid robot from URDF into Gazebo. Robot spawns 1m above ground and falls due to gravity.

## Usage
```bash
cd ~/hackathon1-humanoid-robotics-book/examples/module-2/chapter-1/example-02-spawn-humanoid
ros2 launch launch/spawn_robot.launch.py
```

## Expected Behavior
- Gazebo opens with empty world
- Humanoid robot spawns at position (0, 0, 1.0)
- Robot falls and lands on ground
- All joints visible and functioning
- RTF ≥ 0.9

## Verification
```bash
# Check robot model is spawned
ros2 service call /gazebo/get_model_state gazebo_msgs/srv/GetModelState "{model_name: 'simple_humanoid'}"
```

## Troubleshooting
- **URDF not found**: Verify file path in launch file
- **Robot falls through ground**: Increase contact stiffness in URDF `<gazebo>` tags
