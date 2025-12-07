# Gazebo Client-Server Architecture

```mermaid
graph TB
    subgraph "Client Layer"
        A[gzclient<br/>Gazebo GUI]
        B[User Input<br/>Mouse/Keyboard]
        C[3D Renderer<br/>OGRE Engine]
    end

    subgraph "Server Layer"
        D[gzserver<br/>Physics Engine]
        E[Physics Core<br/>ODE/Bullet/DART]
        F[Sensor Manager<br/>Ray/Camera/IMU]
        G[Plugin System<br/>ROS 2 Bridges]
    end

    subgraph "ROS 2 Integration"
        H[ROS 2 Topics<br/>/joint_states<br/>/cmd_vel<br/>/scan]
        I[ROS 2 Services<br/>/spawn_entity<br/>/apply_wrench]
        J[ROS 2 Nodes<br/>Controllers<br/>Planners]
    end

    subgraph "World Data"
        K[SDF World Files<br/>.world]
        L[URDF Robot Models<br/>.urdf]
        M[Model Database<br/>ground_plane<br/>obstacles]
    end

    B --> A
    A <-->|Scene Updates<br/>Render Requests| D
    A --> C
    C -->|Visual Output| A

    D --> E
    D --> F
    D --> G

    G <-->|Publish/Subscribe| H
    G <-->|Call/Response| I
    H <-->|Data Exchange| J
    I <-->|Commands| J

    K -->|Load| D
    L -->|Spawn| D
    M -->|Insert| D

    E -->|State Updates<br/>60-1000 Hz| D
    F -->|Sensor Data| G

    style A fill:#e1f5ff
    style D fill:#ffe1e1
    style E fill:#ffebe1
    style G fill:#e1ffe1
    style H fill:#f0e1ff

    classDef clientClass fill:#e1f5ff,stroke:#333,stroke-width:2px
    classDef serverClass fill:#ffe1e1,stroke:#333,stroke-width:2px
    classDef rosClass fill:#f0e1ff,stroke:#333,stroke-width:2px

    class A,B,C clientClass
    class D,E,F,G serverClass
    class H,I,J rosClass
```

## Architecture Components

### Client Layer (gzclient)
- **Gazebo GUI**: User interface for visualization and interaction
- **OGRE Renderer**: OpenGL rendering engine for 3D graphics
- **User Input**: Mouse/keyboard controls for camera, model manipulation

### Server Layer (gzserver)
- **Physics Engine**: Simulates rigid body dynamics, contacts, forces
  - ODE (default): Constraint-based solver, fast for articulated robots
  - Bullet: Impulse-based solver, good general-purpose
  - DART: High-accuracy solver, smoother contacts
- **Sensor Manager**: Simulates LiDAR, cameras, IMU, contact sensors
- **Plugin System**: Extensibility via C++ plugins, ROS 2 bridges

### ROS 2 Integration
- **Topics**: Publish/subscribe for continuous data streams
  - `/joint_states`: Robot joint positions, velocities
  - `/cmd_vel`: Velocity commands for mobile robots
  - `/scan`: LiDAR point clouds
- **Services**: Request/response for one-time operations
  - `/spawn_entity`: Spawn URDF models
  - `/apply_body_wrench`: Apply forces to links
- **Nodes**: Control algorithms, motion planners, state estimators

### World Data
- **SDF World Files**: XML descriptions of simulation environments
- **URDF Robot Models**: Unified Robot Description Format (links, joints, sensors)
- **Model Database**: Pre-built models (ground_plane, boxes, humanoids)

## Communication Flow

1. **Initialization**:
   - `gzserver` loads SDF world file
   - Spawns models (URDF converted to SDF internally)
   - Initializes physics engine and sensors

2. **Simulation Loop** (1000 Hz default):
   - Physics engine updates: $\mathbf{x}_{t+1} = f(\mathbf{x}_t, \mathbf{u}_t, \Delta t)$
   - Sensors collect data (ray casting, rendering)
   - Plugins publish ROS 2 messages

3. **Visualization** (60 Hz):
   - `gzserver` sends scene updates to `gzclient`
   - `gzclient` renders 3D view via OGRE

4. **User Interaction**:
   - User inputs (mouse drag, keyboard) → `gzclient`
   - `gzclient` sends commands → `gzserver`
   - `gzserver` applies forces/torques, updates state

## Separation Benefits

**Headless Operation**: Run `gzserver` without `gzclient` for:
- Faster-than-real-time training (RTF > 1.0)
- Remote simulation on GPU servers
- Batch testing with multiple parallel instances

**Distributed Simulation**:
- `gzserver` on powerful workstation
- `gzclient` on lightweight laptop
- Connected via network (TCP/IP)

---

**Usage in Chapter 1**:
- Section 2: "Understanding Gazebo Architecture" (referenced in chapter-1.mdx:95-110)
- Explains why RTF measurement appears in `gzclient` but physics runs in `gzserver`
