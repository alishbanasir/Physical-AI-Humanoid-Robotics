---
title: "1.7 Exercises and Summary"
description: "Hands-on exercises, troubleshooting guide, further reading, and Chapter 1 summary"
---

# 1.7 Exercises and Summary

## Exercises

Complete the following exercises to reinforce your Chapter 1 knowledge. Exercises progress from beginner to advanced difficulty.

---

### Exercise 1: Isaac Sim Installation and Sample Robot (Beginner)

**Difficulty**: ⭐ Beginner
**Estimated Time**: 30-45 minutes

**Objective**: Successfully install Isaac Sim and verify installation by loading and simulating a sample robot.

**Requirements**:
1. Install Isaac Sim 2023.1 LTS following Section 1-2 instructions
2. Launch Isaac Sim and load the Franka sample robot (`Isaac/Samples/Robots/franka.usd`)
3. Run simulation and observe physics behavior
4. Measure Real-Time Factor (RTF) - should be ≥1.0 in headless mode

**Acceptance Criteria**:
- [ ] Isaac Sim 2023.1.1 launches without errors
- [ ] Franka robot loads and displays in viewport
- [ ] Simulation runs smoothly (robot responds to gravity)
- [ ] RTF ≥1.0 confirmed via measurement

**Hints**:
<details>
<summary>Click to expand hints</summary>

- Verify NVIDIA driver ≥525 with `nvidia-smi`
- First launch takes 2-5 minutes (shader compilation)
- For headless RTF test: `./isaac-sim.sh --headless` then measure simulation time vs wall-clock time
- If RTF &lt;1.0, reduce scene complexity or check GPU utilization with `nvidia-smi dmon`

</details>

---

### Exercise 2: Import Your Module 1 Humanoid to Isaac Sim (Beginner)

**Difficulty**: ⭐ Beginner
**Estimated Time**: 30-60 minutes

**Objective**: Convert your Module 1 humanoid URDF to USD format and simulate in Isaac Sim.

**Requirements**:
1. Locate your humanoid URDF from Module 1 (or use provided sample)
2. Enable URDF Importer extension in Isaac Sim
3. Import URDF via GUI (Section 1-3)
4. Verify articulation structure, joint limits, and collision geometry
5. Run simulation and ensure robot stands/falls gracefully under gravity

**Acceptance Criteria**:
- [ ] URDF imports without errors
- [ ] USD stage hierarchy shows all links and joints correctly
- [ ] Robot simulates without exploding or excessive jitter
- [ ] Joint limits match URDF specifications (verify via Property panel)
- [ ] USD file saved for future use

**Hints**:
<details>
<summary>Click to expand hints</summary>

- Use Section 1-3 validation checklist
- If robot explodes, increase PhysX solver iterations to 16-32 (**Physics → Settings**)
- Ensure all mesh files referenced in URDF are present
- Check collision geometry visible with **Show → Physics → Colliders**

</details>

---

### Exercise 3: Create Custom Humanoid Environment (Intermediate)

**Difficulty**: ⭐⭐ Intermediate
**Estimated Time**: 1-2 hours

**Objective**: Build a custom USD scene with your humanoid robot, terrain, and obstacles for navigation testing.

**Requirements**:
1. Create USD stage with ground plane
2. Spawn your humanoid robot from Exercise 2 at origin (0, 0, 1) - slightly above ground
3. Add 3+ obstacles (boxes, cylinders, or imported meshes) placed strategically
4. Configure PhysX scene parameters: gravity -9.81 m/s², simulation frequency 60 Hz
5. Save scene as `humanoid_environment.usd`

**Acceptance Criteria**:
- [ ] Scene contains ground plane with collision
- [ ] Humanoid robot spawned at specified position
- [ ] At least 3 distinct obstacles placed in scene
- [ ] PhysX configured correctly (robot falls to ground, collides with obstacles)
- [ ] Scene saved and reloadable

**Hints**:
<details>
<summary>Click to expand hints</summary>

- Use **Create → Mesh → Plane** for ground
- Use **Create → Mesh → Cube/Cylinder** for obstacles
- Apply collision properties to all meshes (**Add → Physics → Collision**)
- Position obstacles using **Transform** properties (X, Y, Z coordinates)
- See code example **04-custom-scene** for Python automation approach

</details>

---

### Exercise 4: Configure RTX Lighting and PBR Materials (Intermediate)

**Difficulty**: ⭐⭐ Intermediate
**Estimated Time**: 1-2 hours

**Objective**: Set up photorealistic rendering with RTX lighting and PBR materials for visualization.

**Requirements**:
1. Load your Exercise 3 scene
2. Add HDR dome light (sky environment)
3. Add 2+ area lights for studio-style lighting
4. Apply PBR materials to robot links:
   - Metallic material for robot body
   - Matte material for joints
   - Glossy material for end effectors
5. Enable RTX ray tracing and capture rendered image

**Acceptance Criteria**:
- [ ] Dome light added with HDR texture
- [ ] Multiple area lights positioned for illumination
- [ ] PBR materials applied to robot (visible differences in metallic/matte/glossy)
- [ ] RTX rendering enabled (photorealistic viewport)
- [ ] Screenshot captured showing rendered scene

**Hints**:
<details>
<summary>Click to expand hints</summary>

- HDR textures available in **Omniverse Asset Store** (search "HDRI sky")
- Create area lights: **Create → Light → Rect Light**
- Access PBR materials: **Omniverse Material Library** or **Create → Material**
- Enable RTX: Viewport top-right → Rendering Mode → **RTX - Interactive**
- See code example **06-rtx-lighting** for Python automation

</details>

---

### Exercise 5: Automate Simulation Tests with Python API (Advanced)

**Difficulty**: ⭐⭐⭐ Advanced
**Estimated Time**: 2-3 hours

**Objective**: Write Python script to automate Isaac Sim tests: spawn 5 robot instances, apply different physics parameters to each, run parallel simulations, collect performance metrics (RTF), save results to CSV.

**Requirements**:
1. Create Python script using `omni.isaac.core` API
2. Spawn 5 humanoid robots in array formation (spaced 2m apart)
3. Apply different PhysX solver iterations to each: [4, 8, 16, 24, 32]
4. Run simulation for 10 seconds (sim time) for each configuration
5. Measure and record RTF for each robot
6. Save results to `performance_results.csv` with columns: `robot_id`, `solver_iterations`, `rtf`, `stable` (boolean: did robot stay upright?)

**Acceptance Criteria**:
- [ ] Script spawns 5 robots automatically
- [ ] Different PhysX configurations applied per robot
- [ ] Simulation runs to completion (10 sim seconds)
- [ ] RTF measured accurately for each configuration
- [ ] CSV file generated with results
- [ ] Analysis: Determine optimal solver iterations for humanoid stability vs performance

**Hints**:
<details>
<summary>Click to expand hints</summary>

- Use `omni.isaac.core.World` for simulation control
- Spawn robots with `World.scene.add()` at different positions
- Set solver iterations via `PhysxSchema.PhysxSceneAPI` per-robot
- Measure RTF: `rtf = simulation_time / wall_clock_time`
- Check stability: Monitor robot base link Z-position (if < threshold → fallen)
- See code example **10-python-automation** for full implementation

</details>

---

## Troubleshooting Guide

### Common Issues and Solutions

#### Issue 1: NVIDIA Driver Version Mismatch

**Symptom**: Isaac Sim fails to launch with error "NVIDIA driver version 525 or higher required"

**Cause**: Outdated NVIDIA driver (< 525)

**Solution**:
```bash
# Ubuntu:
sudo ubuntu-drivers install nvidia-driver-525
sudo reboot
# Verify:
nvidia-smi
```

---

#### Issue 2: Isaac Sim Installation Errors

**Symptom**: Omniverse Launcher shows "Installation failed" or download stalls at 50%

**Cause**: Insufficient disk space (< 50GB free), network interruption, or corrupted partial download

**Solution**:
```bash
# Check disk space
df -h

# Clear Omniverse cache and retry
rm -rf ~/.local/share/ov/Cache
rm -rf ~/.local/share/ov/pkg/isaac_sim-*  # Remove partial installs

# Relaunch Omniverse Launcher, reinstall Isaac Sim 2023.1.1
```

---

#### Issue 3: USD Import Failures (URDF)

**Symptom**: URDF files fail to import with error "URDF Importer extension not found" or "Parsing error"

**Cause**: (1) URDF Importer extension not enabled, or (2) Invalid URDF XML syntax / missing mesh files

**Solution**:
1. Enable extension: **Window** → **Extensions** → Search "URDF Importer" → Toggle ON
2. Validate URDF syntax:
   ```bash
   check_urdf /path/to/humanoid.urdf
   ```
3. Ensure all mesh files (.stl, .dae, .obj) referenced in URDF `<mesh filename="...">` exist and paths are correct

---

#### Issue 4: ROS 2 Bridge Not Publishing Topics

**Symptom**: `ros2 topic list` shows no topics from Isaac Sim, or `/joint_states` missing

**Cause**: ROS 2 bridge extension not enabled, or ROS 2 not installed on system

**Solution**:
1. Enable extension: **Window** → **Extensions** → Search "ROS2 Bridge" → Toggle ON
2. Verify ROS 2 Humble installed:
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 topic list
   ```
3. In Isaac Sim, check ROS 2 bridge is active: Look for ROS 2 icon in Extensions Manager
4. Ensure simulation is running (Press ▶️ Play) - topics only publish during simulation

---

#### Issue 5: RTX Rendering Slow / Low Viewport FPS

**Symptom**: Viewport FPS < 10, interactive editing feels sluggish

**Cause**: GPU cannot handle real-time ray tracing at high resolution, or complex scene

**Solution**:
- Reduce viewport resolution: **Edit** → **Preferences** → **Rendering** → Set to 1280x720 or 960x540
- Switch to rasterization mode: Viewport top-right → Rendering Mode → **Iray - Iterative** (non-ray-traced preview)
- Simplify scene: Reduce polygon count of meshes, limit number of lights (3-5 max for real-time)
- Use **headless mode** when visualization not needed: `./isaac-sim.sh --headless` (maximizes RTF)

---

#### Issue 6: PhysX Simulation Instability (Robot Explodes or Jitters)

**Symptom**: Robot limbs fly apart, vibrate violently, or entire robot "explodes" on simulation start

**Cause**: PhysX solver parameters insufficient for complex articulated robot (default solver iterations=4 too low for humanoids)

**Solution**:
1. Increase solver iterations: **Physics** → **Settings** → Set **"Position Iteration Count"** to 16-32 (default is 4)
2. Reduce simulation timestep: Set **"Physics Update Rate"** to 120 Hz (from default 60 Hz) for finer time resolution
3. Check contact stiffness: **Physics** → Select robot link → **Physics Material** → Increase contact stiffness to 10,000-50,000
4. Verify mass distribution: Unrealistic inertia tensors cause instability - ensure URDF masses/inertias are correct

**Parameter Tuning Guidance**:
- **Humanoid robots**: Solver iterations 16-32, Physics rate 120 Hz
- **Wheeled robots**: Solver iterations 8-16, Physics rate 60 Hz
- **Manipulator arms (fixed base)**: Solver iterations 4-8, Physics rate 60 Hz

---

## Screenshot Documentation (For Human Reviewers)

The following screenshots need to be captured during actual Isaac Sim usage and placed in `static/img/module-3/isaac-sim-screenshots/`:

1. `omniverse-launcher-download-page.png` - Omniverse website download page
2. `omniverse-launcher-signin.png` - Launcher sign-in screen
3. `isaac-sim-installation-progress.png` - Launcher showing Isaac Sim download progress bar
4. `nvidia-smi-driver-check.png` - Terminal output showing NVIDIA driver version ≥525
5. `isaac-sim-first-launch.png` - Isaac Sim main window with default empty scene
6. `isaac-sim-sample-robot-simulation.png` - Franka robot simulation running, showing physics
7. `nucleus-local-setup.png` - Nucleus server configuration panel in Launcher
8. `urdf-importer-extension-panel.png` - Extensions window with URDF Importer enabled
9. `urdf-importer-settings.png` - URDF Importer configuration window
10. `usd-stage-hierarchy.png` - Stage panel showing humanoid articulation tree structure
11. `physx-scene-settings.png` - Physics settings panel showing solver iterations, gravity
12. `rtx-lighting-setup.png` - Scene with HDR dome light and area lights configured
13. `pbr-material-editor.png` - Material properties panel showing metallic/roughness parameters
14. `ros2-bridge-extension-enabled.png` - ROS 2 Bridge extension active in Extensions Manager
15. `rviz-joint-states-visualization.png` - RViz showing /joint_states topic from Isaac Sim
16. `python-script-terminal-output.png` - Terminal showing Python API script execution output
17. `rtf-performance-metrics.png` - Performance measurement results showing RTF ≥1.0
18. `isaac-sim-custom-scene-render.png` - Custom humanoid environment with photorealistic RTX rendering

---

## Further Reading

Expand your Isaac Sim knowledge with these curated resources:

1. **[NVIDIA Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html)** - Official documentation covering installation, core API, robot setup, and advanced features. Focus on:
   - **Core API**: `omni.isaac.core` module for programmatic control
   - **Robot Setup**: Articulation configuration and joint control
   - **Sensors**: Camera, LiDAR, and IMU configuration

2. **[Omniverse USD Documentation](https://docs.omniverse.nvidia.com/usd/latest/index.html)** - Universal Scene Description fundamentals. Focus on:
   - **USD Basics**: Prims, attributes, and relationships
   - **Composition**: References, layers, and variants
   - **Physics Schema**: UsdPhys API for rigid body dynamics

3. **[PhysX 5 SDK Documentation](https://nvidia-omniverse.github.io/PhysX/physx/5.3.1/index.html)** - Detailed physics engine reference. Focus on:
   - **Rigid Body Dynamics**: Mass, inertia, forces, and torques
   - **Articulations**: Reduced-coordinate representation for robots
   - **Collision Detection**: Broadphase, narrowphase, and contact generation

4. **[ROS 2 Humble Documentation](https://docs.ros.org/en/humble/index.html)** - ROS 2 fundamentals and bridge architectures. Focus on:
   - **Topics and Messages**: Publishing and subscribing
   - **Launch Files**: Automating node startup
   - **TF2 Transforms**: Coordinate frame management

5. **Research Paper**: "GPU-Accelerated Robotic Simulation for Massively Parallel Reinforcement Learning" (NVIDIA Isaac Gym, NeurIPS 2021) - Describes architecture for 1000+ parallel simulations on single GPU, foundational concepts for Chapter 3 RL training

---

## Chapter 1 Summary

Congratulations on completing Chapter 1! You've built the foundation for GPU-accelerated robotics simulation with NVIDIA Isaac Sim.

**Key Skills Learned**:
- **Installation**: Set up Isaac Sim 2023.1 LTS on local or cloud GPU infrastructure
- **URDF-to-USD Conversion**: Transformed Module 1 humanoid robots to Isaac Sim native format while preserving kinematics and dynamics
- **Scene Creation**: Built custom USD environments with terrain, obstacles, and realistic physics properties
- **Physics Configuration**: Tuned PhysX 5 parameters (solver iterations, contact properties) for stable humanoid simulation achieving RTF ≥1.0
- **RTX Rendering**: Configured photorealistic lighting with HDR skyboxes, area lights, and PBR materials for visualization
- **ROS 2 Integration**: Enabled bidirectional communication between Isaac Sim and ROS 2 for sensor data publishing and robot control
- **Python Automation**: Automated simulation workflows using `omni.isaac.core` API for batch testing and data collection

**Importance in Physical AI Pipeline**: Isaac Sim serves as the high-fidelity simulation environment for the entire Module 3 workflow. GPU-accelerated physics (PhysX 5) enables real-time simulation of complex humanoid robots, while RTX rendering provides photorealistic camera sensors for vision-based tasks. This foundation enables:
- **Chapter 2**: GPU-accelerated perception pipelines (Isaac ROS) processing Isaac Sim sensor data
- **Chapter 3**: Massively parallel RL training (Isaac Lab) with 1000+ simulated humanoid environments
- **Chapter 4**: Autonomous navigation testing (Nav2) in Isaac Sim environments before real-world deployment

**Why GPU Acceleration Matters**: Traditional CPU-based simulators (Gazebo Classic) achieve RTF 0.5-2.0 for single robots. Isaac Sim on RTX GPUs achieves RTF 2.0-10.0 for single robots, and can run 1000+ robots in parallel for RL training—a paradigm shift enabling practical deep reinforcement learning for humanoid control.

---

## Preview: Chapter 2 - Hardware-Accelerated Perception

Chapter 2 builds on your Isaac Sim foundation by introducing **NVIDIA Isaac ROS**, a collection of GPU-accelerated perception packages. You'll learn to:
- Configure stereo cameras and depth sensors in Isaac Sim for Isaac ROS input
- Process depth images at >30 FPS using CUDA-accelerated stereo vision algorithms
- Run visual SLAM (nvSLAM) achieving &lt;50ms update latency for real-time localization
- Execute DNN-based object detection and semantic segmentation at >30 FPS using TensorRT inference
- Fuse visual-inertial odometry (camera + IMU) for robust pose estimation
- Profile and optimize perception pipelines to maximize GPU utilization

By combining Isaac Sim (simulation) with Isaac ROS (perception), you'll have a complete testing environment for vision-based humanoid robotics—no physical hardware required until deployment!

---

**Ready for Chapter 2?** → [Chapter 2: Hardware-Accelerated Perception with Isaac ROS](../chapter-2-isaac-ros-perception/index.md)

Or continue practicing Chapter 1 skills with the 10+ code examples in `examples/module-3/chapter-1-isaac-sim/`!
