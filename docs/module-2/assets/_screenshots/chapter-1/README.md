# Chapter 1 Screenshots Guide

This directory contains screenshots for Chapter 1: Gazebo Physics Simulation. Follow the instructions below to capture high-quality screenshots for educational use.

---

## Required Screenshots

### 1. **Gazebo GUI with Humanoid Robot**
**Filename**: `gazebo-gui-humanoid.png`

**Purpose**: Show students the complete Gazebo interface with the simple humanoid robot loaded.

**Capture Instructions**:
1. Launch Gazebo with Example 2:
   ```bash
   cd examples/module-2/chapter-1/example-02-spawn-humanoid
   ros2 launch launch/spawn_robot.launch.py
   ```

2. Wait for Gazebo to fully load (robot should be standing on ground)

3. **Camera Position**:
   - Distance: ~3 meters from robot
   - Angle: 30° above horizon
   - Orientation: Front-left view (see robot's face and left side)

4. **UI Elements to Show**:
   - ✅ Left panel: Model tree (expand "simple_humanoid" to show links)
   - ✅ Bottom panel: Timeline (paused or running)
   - ✅ Bottom-right: **Real-Time Factor (RTF)** display - CRITICAL
   - ✅ Top toolbar: Insert, Delete, Transform tools visible
   - ✅ Scene: Ground plane grid visible

5. **Viewport Settings**:
   - Enable grid (View → Grid)
   - Enable shadows (World → Scene → Shadows)
   - Camera mode: Orbit

6. **Screenshot Dimensions**: 1920×1080 (Full HD)

7. **Annotations** (add using image editor after capture):
   - Red box around RTF display with label: "Real-Time Factor: Check ≥ 0.9"
   - Arrow pointing to model tree: "Robot Links"
   - Arrow pointing to ground plane: "Physics-enabled ground"

**Reference**: Chapter 1, Section 2 "Understanding Gazebo Architecture" (chapter-1.mdx:95-110)

---

### 2. **Gravity Demonstration (Robot Falling)**
**Filename**: `gravity-demo.png`

**Purpose**: Demonstrate physics simulation in action - robot falling and landing.

**Capture Instructions**:
1. Launch Gazebo with Example 2:
   ```bash
   cd examples/module-2/chapter-1/example-02-spawn-humanoid
   ros2 launch launch/spawn_robot.launch.py
   ```

2. **Modify Spawn Height**: Edit `launch/spawn_robot.launch.py` temporarily:
   ```python
   # Change -z argument from 1.0 to 2.0
   arguments=['-entity', 'simple_humanoid', '-file', urdf_path,
              '-x', '0', '-y', '0', '-z', '2.0'],  # Spawn 2m high
   ```

3. **Screenshot Sequence** (capture 3 frames, create composite):
   - **Frame 1 (t=0s)**: Robot at spawn position (2m height), not yet falling
   - **Frame 2 (t=0.5s)**: Robot mid-fall, legs extended
   - **Frame 3 (t=1.5s)**: Robot landed on ground, settled

4. **Camera Settings**:
   - Fixed camera position (do not move between frames)
   - Side view (see full robot profile)
   - Distance: 4 meters

5. **Create Composite Image** (using image editor):
   - 3 panels side-by-side: "Before" | "During" | "After"
   - Add time labels: t=0s, t=0.5s, t=1.5s
   - Add motion blur effect (optional) for "During" panel

6. **Annotations**:
   - Arrow showing direction of gravity (-Z axis)
   - Label: "Gravity: -9.81 m/s²"
   - Velocity vector on Frame 2 (show increasing downward velocity)

**Reference**: Chapter 1, Section 4.3 "Spawning Robot from URDF" (chapter-1.mdx:180-220)

---

### 3. **Physics Parameter Tuning (RTF Comparison)**
**Filename**: `rtf-performance.png`

**Purpose**: Show how physics parameters affect Real-Time Factor performance.

**Capture Instructions**:
1. Create 3 different world configurations:

   **Config A: Fast (Low Accuracy)**
   ```xml
   <ode>
     <solver><iters>20</iters></solver>
   </ode>
   ```

   **Config B: Balanced (Default)**
   ```xml
   <ode>
     <solver><iters>50</iters></solver>
   </ode>
   ```

   **Config C: Accurate (Slow)**
   ```xml
   <ode>
     <solver><iters>200</iters></solver>
   </ode>
   ```

2. **Capture Procedure** (for each config):
   - Launch Gazebo with config
   - Let simulation run for 30 seconds
   - Screenshot showing bottom-right RTF display

3. **Create Comparison Table** (using image editor):
   ```
   +-------------------+---------------------+---------------------+
   | Fast (iters=20)   | Balanced (iters=50) | Accurate (iters=200)|
   | RTF: 1.25         | RTF: 0.95           | RTF: 0.65           |
   +-------------------+---------------------+---------------------+
   ```

4. **Annotations**:
   - Green checkmark on "Balanced" (recommended)
   - Note: "Higher iterations → Better accuracy, Lower RTF"
   - Trade-off arrow: Accuracy ↔ Performance

**Reference**: Chapter 1, Section 5.4 "Physics Parameter Tuning" (chapter-1.mdx:280-310)

---

### 4. **Contact Visualization (Optional)**
**Filename**: `contact-forces.png`

**Purpose**: Show collision contacts and forces in Gazebo GUI.

**Capture Instructions**:
1. Launch Example 2 (humanoid standing on ground)

2. **Enable Contact Visualization**:
   - View → Contacts (checkbox)
   - Contacts appear as small spheres at contact points

3. **Camera Position**:
   - Close-up of left foot on ground
   - Zoom to see contact points clearly

4. **Screenshot Elements**:
   - ✅ Pink/magenta spheres at foot-ground contacts
   - ✅ Robot foot geometry visible
   - ✅ Ground plane visible

5. **Annotations**:
   - Arrow pointing to contact spheres: "Contact Points"
   - Label: "Normal Force: ~245 N (half body weight)"
   - Coordinate frame overlay (X-Y-Z axes)

**Reference**: Chapter 1, Section 3.2 "Contact Mechanics" (chapter-1.mdx:115-130)

---

## Screenshot Quality Standards

### Technical Requirements
- **Resolution**: 1920×1080 minimum (Full HD)
- **Format**: PNG (lossless compression)
- **Color Depth**: 24-bit RGB
- **File Size**: <2 MB per image (optimize with TinyPNG or similar)

### Content Guidelines
- **Clarity**: All text must be readable at 100% zoom
- **Lighting**: Ensure good contrast, avoid overly dark/bright areas
- **Focus**: Key elements (RTF, robot, contacts) should be in focus
- **Consistency**: Use same Gazebo theme across all screenshots

### Accessibility (Alt Text)
Each screenshot MUST have alt text in the MDX file:

```mdx
![Gazebo GUI showing simple humanoid robot standing on ground plane with model tree visible on left and RTF=0.95 displayed in bottom-right corner](./assets/_screenshots/chapter-1/gazebo-gui-humanoid.png)
```

---

## Integration into Chapter 1

### Where to Insert Screenshots

**In `docs/module-2/chapter-1.mdx`**, add image references:

```mdx
<!-- After Section 2: Understanding Gazebo Architecture -->
![Gazebo GUI Overview](./assets/_screenshots/chapter-1/gazebo-gui-humanoid.png)
*Figure 1: Gazebo GUI with simple humanoid robot loaded. Note the Real-Time Factor (RTF) in the bottom-right corner.*

<!-- After Section 4.3: Spawning Robot from URDF -->
![Gravity Simulation](./assets/_screenshots/chapter-1/gravity-demo.png)
*Figure 2: Robot falling due to gravity (left to right: t=0s, t=0.5s, t=1.5s). Demonstrates physics engine integrating Newton's laws.*

<!-- After Example 4: Physics Parameter Tuning -->
![RTF Performance Comparison](./assets/_screenshots/chapter-1/rtf-performance.png)
*Figure 3: Real-Time Factor comparison across solver iteration counts. Higher iterations improve accuracy at the cost of performance.*

<!-- Optional: After Section 3.2: Contact Mechanics -->
![Contact Visualization](./assets/_screenshots/chapter-1/contact-forces.png)
*Figure 4: Contact points (pink spheres) between humanoid foot and ground plane. Enable via View → Contacts in Gazebo GUI.*
```

---

## Validation Checklist

Before marking screenshots as complete, verify:

- [ ] **Screenshot 1** (Gazebo GUI): RTF display visible and readable
- [ ] **Screenshot 2** (Gravity Demo): 3-panel composite shows clear progression
- [ ] **Screenshot 3** (RTF Comparison): RTF values clearly visible in all 3 configs
- [ ] **Screenshot 4** (Contacts - Optional): Contact spheres visible
- [ ] All images optimized (file size < 2 MB)
- [ ] Alt text added to all `<img>` tags in chapter-1.mdx
- [ ] Images display correctly in Docusaurus build (`npm run build`)
- [ ] Annotations (arrows, labels) professionally added

---

## Tools Recommended

**Screenshot Capture**:
- Linux: GNOME Screenshot (`gnome-screenshot`), Flameshot
- Windows: Snipping Tool, ShareX
- macOS: Cmd+Shift+4

**Image Editing**:
- GIMP (free, cross-platform) - for annotations
- Inkscape (free, cross-platform) - for vector arrows
- Figma (web-based) - for composite layouts

**Optimization**:
- TinyPNG (web) - PNG compression
- OptiPNG (CLI) - Lossless optimization
- ImageMagick (CLI) - Batch processing

---

## Example Capture Command (Linux)

```bash
# Launch Gazebo
ros2 launch example-02-spawn-humanoid/launch/spawn_robot.launch.py &

# Wait 5 seconds for full load
sleep 5

# Capture screenshot (requires gnome-screenshot)
gnome-screenshot -w -f gazebo-gui-humanoid.png

# Optimize
optipng -o7 gazebo-gui-humanoid.png

# Move to assets directory
mv gazebo-gui-humanoid.png docs/module-2/assets/_screenshots/chapter-1/
```

---

**Status**: ⏸️ **Pending** - Screenshots to be captured by user or team member with Gazebo environment

**Priority**: Medium (improves learning, but chapter text is self-sufficient)

**Estimated Time**: 1 hour (capture + edit + integrate)
