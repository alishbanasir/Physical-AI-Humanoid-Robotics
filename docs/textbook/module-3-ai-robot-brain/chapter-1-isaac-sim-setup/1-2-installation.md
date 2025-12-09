---
title: "1.2 Installation and Setup"
description: "Step-by-step guide to installing NVIDIA Isaac Sim 2023.1 LTS on Ubuntu/Windows, configuring Nucleus server, and setting up cloud GPU instances"
---

# 1.2 Installation and Setup

##Introduction

This section guides you through installing **NVIDIA Isaac Sim 2023.1 LTS** on your local machine or cloud GPU instance. We target the 2023.1 LTS (Long-Term Support) release for stability and compatibility with RTX 2060+ GPUs. Migration notes for Isaac Sim 2024.x users are provided at the end.

---

## System Requirements

### Minimum Hardware

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| **GPU** | NVIDIA RTX 2060 (6GB VRAM) | NVIDIA RTX 3080 (10GB+ VRAM) |
| **RAM** | 16GB | 32GB (for large scenes or RL training) |
| **CPU** | Intel i7 / AMD Ryzen 7 (6 cores) | Intel i9 / AMD Ryzen 9 (8+ cores) |
| **Storage** | 50GB free SSD space | 100GB+ SSD (for assets and logs) |
| **OS** | Ubuntu 22.04 LTS or Windows 10 | Ubuntu 22.04 LTS (best performance) |

### Software Prerequisites

- **NVIDIA Driver**: Version 525+ (check with `nvidia-smi`)
- **Python**: 3.10+ (Isaac Sim 2023.1 uses Python 3.10)

---

## Installation Methods

Choose one of the following installation methods:

1. **Local Installation** (Ubuntu/Windows) - Full control, best performance
2. **Cloud GPU** (AWS G4/G5) - No local GPU required, pay-per-use
3. **NVIDIA LaunchPad** (Free Trial) - Free exploration, limited hours

---

## Method 1: Local Installation (Ubuntu 22.04)

### Step 1: Install NVIDIA Driver

```bash
# Check current driver version
nvidia-smi

# If version < 525, install updated driver
sudo ubuntu-drivers list
sudo ubuntu-drivers install nvidia-driver-525

# Reboot to load new driver
sudo reboot

# Verify driver installation
nvidia-smi
# Expected output: Driver Version: 525.xx or higher
```

**Screenshot Placeholder**: `nvidia-smi-driver-check.png` - Terminal showing NVIDIA driver 525+ with GPU info

### Step 2: Download and Install Omniverse Launcher

1. Visit [NVIDIA Omniverse](https://www.nvidia.com/en-us/omniverse/)
2. Click "Download Omniverse Launcher" (requires free NVIDIA account)
3. Download for Ubuntu: `omniverse-launcher-linux.AppImage`

**Screenshot Placeholder**: `omniverse-launcher-download-page.png` - Omniverse website download page

4. Make executable and run:

```bash
chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage
```

5. Sign in with your NVIDIA account
6. Accept terms and complete installation

**Screenshot Placeholder**: `omniverse-launcher-signin.png` - Launcher sign-in screen

### Step 3: Install Isaac Sim 2023.1 LTS via Launcher

1. In Omniverse Launcher, navigate to **"Exchange"** tab
2. Search for "Isaac Sim"
3. Find **"Isaac Sim 2023.1.1"** (LTS release)
4. Click **"Install"** button
5. Choose installation directory (default: `~/.local/share/ov/pkg/isaac_sim-2023.1.1`)
6. Wait for download (~20GB, 15-60 minutes depending on connection)

**Screenshot Placeholder**: `isaac-sim-installation-progress.png` - Launcher showing Isaac Sim download progress

### Step 4: Configure Nucleus Server (Local)

Nucleus server is optional for single-user setups, but useful for asset management.

**Option A: Local Nucleus (Lightweight)**

1. In Omniverse Launcher, go to **"Nucleus"** tab
2. Click **"Add Local Nucleus Service"**
3. Set administrator password
4. Click **"Create"**
5. Nucleus starts at `localhost:3030` or `localhost:3009`

**Option B: Skip Nucleus (Use Local Files)**

You can work entirely with local USD files (`file:///home/user/my_scene.usd`) without Nucleus. This is fine for learning and single-user projects.

**Screenshot Placeholder**: `nucleus-local-setup.png` - Nucleus configuration in Launcher

### Step 5: Launch Isaac Sim and Verify Installation

```bash
# Navigate to Isaac Sim installation directory
cd ~/.local/share/ov/pkg/isaac_sim-2023.1.1

# Launch Isaac Sim GUI
./isaac-sim.sh

# Alternative: Launch headless (no GUI, for testing)
./isaac-sim.sh --headless
```

**First Launch**: Isaac Sim may take 2-5 minutes to initialize shaders and cache. Subsequent launches are faster (~30 seconds).

**Screenshot Placeholder**: `isaac-sim-first-launch.png` - Isaac Sim main window with default scene

### Step 6: Load Sample Scene for Verification

1. In Isaac Sim window, go to **File** → **Open**
2. Navigate to `Isaac/Samples/Robots/` (in Nucleus browser or local samples)
3. Open `simple_robot.usd` or `franka.usd`
4. Click **Play** button (▶️) to start simulation
5. Verify physics simulation runs smoothly (robot responds to gravity)
6. Check viewport FPS (should be 30-60 FPS for simple scenes)

**Measure Real-Time Factor (RTF)**:
- Run simulation for 60 seconds (sim time)
- Measure wall-clock time elapsed
- Calculate RTF = 60 / wall_clock_time
- **Target**: RTF ≥1.0 (e.g., 60 sim seconds in 30 wall-clock seconds → RTF = 2.0)

**Screenshot Placeholder**: `isaac-sim-sample-robot-simulation.png` - Franka robot simulation running

### Step 7: Verify Python API Access

```bash
# Isaac Sim includes standalone Python interpreter
./python.sh

# In Python shell:
>>> import omni.isaac.core
>>> print(omni.isaac.core.__version__)
# Expected: Version string like "2023.1.1"
>>> exit()
```

**Installation Verification Checklist**:
- ✅ NVIDIA driver 525+ installed
- ✅ Omniverse Launcher running
- ✅ Isaac Sim 2023.1.1 installed and launches
- ✅ Sample robot simulates with RTF ≥1.0
- ✅ Python API accessible

**If all checks pass, your local installation is complete!**

---

## Method 2: Cloud GPU Installation (AWS EC2)

If you don't have a local NVIDIA GPU, AWS EC2 provides on-demand and spot instances with RTX-class GPUs.

### Step 1: Choose Instance Type

| Instance Type | GPU | VRAM | vCPUs | RAM | On-Demand Price | Spot Price (est.) |
|---------------|-----|------|-------|-----|-----------------|-------------------|
| **g4dn.xlarge** | NVIDIA T4 | 16GB | 4 | 16GB | ~$0.526/hr | ~$0.15/hr |
| **g4dn.2xlarge** | NVIDIA T4 | 16GB | 8 | 32GB | ~$0.752/hr | ~$0.22/hr |
| **g5.xlarge** | NVIDIA A10G | 24GB | 4 | 16GB | ~$1.006/hr | ~$0.30/hr |
| **g5.2xlarge** | NVIDIA A10G | 24GB | 8 | 32GB | ~$1.212/hr | ~$0.36/hr |

**Recommendation**: Start with **g4dn.xlarge** for learning (sufficient for single robot RTF ≥1.0). Upgrade to **g5.xlarge** for RL training with 2048 parallel environments.

### Step 2: Launch EC2 Instance

```bash
# Using AWS CLI (or use AWS Console web interface)

# 1. Create key pair for SSH access
aws ec2 create-key-pair --key-name isaac-sim-key --query 'KeyMaterial' --output text > isaac-sim-key.pem
chmod 400 isaac-sim-key.pem

# 2. Launch instance with Ubuntu 22.04 Deep Learning AMI
# (Deep Learning AMI includes NVIDIA drivers pre-installed)
aws ec2 run-instances \
    --image-id ami-0c55b159cbfafe1f0 \  # Ubuntu 22.04 DL AMI (us-east-1, check for your region)
    --instance-type g4dn.xlarge \
    --key-name isaac-sim-key \
    --security-group-ids sg-xxxxxx \  # Configure security group with SSH (port 22) access
    --block-device-mappings '[{"DeviceName":"/dev/sda1","Ebs":{"VolumeSize":100}}]'  # 100GB storage

# 3. Get instance public IP
aws ec2 describe-instances --query 'Reservations[0].Instances[0].PublicIpAddress'

# 4. SSH into instance
ssh -i isaac-sim-key.pem ubuntu@<PUBLIC_IP>
```

**Cost Optimization**: Use **Spot Instances** for 70% discount (terminates if AWS needs capacity back, but you can save checkpoint and resume). For training jobs:

```bash
aws ec2 request-spot-instances \
    --spot-price "0.30" \  # Max price you're willing to pay
    --instance-count 1 \
    --type "one-time" \
    --launch-specification file://spot-spec.json
```

### Step 3: Install Isaac Sim on Cloud Instance

```bash
# On EC2 instance (via SSH):

# 1. Verify GPU
nvidia-smi
# Should show T4 or A10G GPU

# 2. Download Omniverse Launcher (headless)
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

# 3. For headless installations, use Isaac Sim Docker container (recommended)
# Pull Isaac Sim Docker image from NVIDIA NGC
docker pull nvcr.io/nvidia/isaac-sim:2023.1.1

# 4. Run Isaac Sim in Docker with GPU passthrough
docker run --gpus all -it \
    -v ~/isaac_sim_data:/workspace/data \  # Mount local directory for data persistence
    nvcr.io/nvidia/isaac-sim:2023.1.1 \
    /bin/bash

# Inside Docker container:
./isaac-sim.sh --headless  # Run headless Isaac Sim
```

**X11 Forwarding for GUI (Optional)**:
If you want to see Isaac Sim GUI from cloud instance:

```bash
# On your local machine, SSH with X11 forwarding:
ssh -X -i isaac-sim-key.pem ubuntu@<PUBLIC_IP>

# On EC2 instance, install X11 libraries:
sudo apt-get update
sudo apt-get install -y xorg

# Launch Isaac Sim with GUI (rendered on cloud, displayed locally):
./isaac-sim.sh
```

**Warning**: X11 forwarding over SSH is slow for interactive use. Use headless mode for training/testing, and download videos/screenshots for review.

### Step 4: Stop Instance When Not in Use

```bash
# Stop instance (preserves storage, stops compute charges)
aws ec2 stop-instances --instance-ids i-xxxxxxxxx

# Start instance later
aws ec2 start-instances --instance-ids i-xxxxxxxxx

# Terminate instance (deletes storage, stops all charges)
aws ec2 terminate-instances --instance-ids i-xxxxxxxxx
```

**Cost Management**:
- Stop instances when not actively training/testing
- Use Spot instances for long-running experiments (can save checkpoints and resume if interrupted)
- Set billing alarms in AWS Console to avoid unexpected charges

---

## Method 3: NVIDIA LaunchPad (Free Trial)

**NVIDIA LaunchPad** offers free GPU access for limited hours per month, perfect for exploring Isaac Sim before committing to paid options.

1. Visit [NVIDIA LaunchPad](https://www.nvidia.com/en-us/data-center/launchpad/)
2. Sign up for free account
3. Request access to **"Isaac Sim"** lab
4. Once approved, launch cloud instance (pre-configured with Isaac Sim)
5. Access via browser (NVIDIA provides streaming GPU desktop)

**Limitations**:
- 8-16 hours/month free tier (varies by promotion)
- Cannot save data persistently (download results before session ends)
- Shared GPU resources (performance may vary)

**Best For**: Trying Isaac Sim before local installation, completing Chapter 1 tutorials, testing compatibility.

---

## Troubleshooting

### Issue 1: NVIDIA Driver Version Mismatch

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

### Issue 2: Isaac Sim Installation Errors

**Symptom**: Omniverse Launcher shows "Installation failed" or download stalls

**Cause**: Insufficient disk space (< 50GB free), network issues, or corrupted download

**Solution**:

```bash
# Check disk space:
df -h

# If low, free up space or choose different installation directory

# Clear Omniverse cache and retry:
rm -rf ~/.local/share/ov/Cache
rm -rf ~/.local/share/ov/pkg/isaac_sim-*  # Remove partial install

# Relaunch Omniverse Launcher and reinstall Isaac Sim
```

### Issue 3: USD Import Failures

**Symptom**: URDF files fail to import with error "URDF Importer extension not found"

**Cause**: URDF Importer extension not enabled

**Solution**:
1. In Isaac Sim, go to **Window** → **Extensions**
2. Search for "URDF Importer"
3. Toggle switch to **ON**
4. Retry URDF import

### Issue 4: ROS 2 Bridge Not Publishing Topics

**Symptom**: `ros2 topic list` shows no topics from Isaac Sim

**Cause**: ROS 2 bridge extension not enabled, or ROS 2 not installed

**Solution**:
1. Enable **"ROS2 Bridge"** extension in Isaac Sim (**Window** → **Extensions** → Search "ROS2 Bridge" → Toggle ON)
2. Verify ROS 2 Humble installed on your system:

```bash
source /opt/ros/humble/setup.bash
ros2 topic list
```

3. Ensure Isaac Sim is publishing to ROS 2 (check extension settings for topic names)

### Issue 5: RTX Rendering Slow / Low FPS

**Symptom**: Viewport FPS < 10, simulation feels sluggish

**Cause**: GPU cannot handle real-time ray tracing at high resolution

**Solution**:
- Reduce viewport resolution: **Edit** → **Preferences** → **Rendering** → Set resolution to 1280x720 or lower
- Disable real-time ray tracing: Change render mode from "RTX - Interactive" to "Iray - Iterative" (or use rasterization mode for faster preview)
- Use **headless mode** (`./isaac-sim.sh --headless`) when rendering is not needed (maximizes RTF)

### Issue 6: PhysX Simulation Instability (Robot Explodes/Jitters)

**Symptom**: Robot limbs fly apart, jitter violently, or simulation crashes

**Cause**: PhysX solver parameters too low for complex articulated robot

**Solution**:
- Increase solver iterations: **Physics** → **Settings** → Set "Position Iteration Count" to 16-32 (default is 4)
- Reduce simulation timestep: Set **"Physics Update Rate"** to 120 Hz (from default 60 Hz)
- Increase contact stiffness: Adjust material physics properties for robot links
- See Section 1-4 for detailed PhysX tuning guidance

---

## Migration Notes for Isaac Sim 2024.x Users

If you're using Isaac Sim 2024.x (latest release) instead of 2023.1 LTS:

**API Changes**:
- Some `omni.isaac.core` API methods renamed or deprecated (check [release notes](https://docs.omniverse.nvidia.com/isaacsim/latest/release_notes.html))
- PhysX 5.3 → 5.4 parameter changes (contact defaults adjusted)

**Compatibility**:
- Code examples in this chapter target 2023.1 API
- Most examples work with 2024.x with minor adjustments
- Look for comments like `# Isaac Sim 2024.x: Use new_method() instead` in example code

**Recommendation**: Stick with 2023.1 LTS for this course to match tested examples. Migrate to 2024.x after completing Module 3.

---

## Verification Checklist

Before proceeding to Section 1-3, ensure:

- [ ] Isaac Sim 2023.1.1 installed and launches successfully
- [ ] Sample robot simulation runs with RTF ≥1.0
- [ ] Python API accessible (`import omni.isaac.core` works)
- [ ] NVIDIA driver version ≥525
- [ ] (Optional) Nucleus server configured or local USD file workflow confirmed
- [ ] (If cloud) SSH access to EC2 instance working, GPU verified with `nvidia-smi`

---

**Next**: [1.3 URDF to USD Conversion →](./1-3-urdf-to-usd.md) - Convert your Module 1 humanoid robot to Isaac Sim USD format!
