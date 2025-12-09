#!/usr/bin/env python3
"""
Example 01: Hello Isaac Sim
Creates a simple scene with a cube, ground plane, and basic physics simulation.

Learning Objectives:
- Initialize Isaac Sim World
- Create basic USD primitives (cube, ground plane)
- Configure PhysX physics
- Run simulation loop
- Measure Real-Time Factor (RTF)

Usage:
    cd ~/.local/share/ov/pkg/isaac_sim-2023.1.1
    ./python.sh /path/to/examples/module-3/chapter-1-isaac-sim/01-hello-isaac-sim/main.py
"""

import time
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid, GroundPlane
from omni.isaac.core.prims import XFormPrim
import numpy as np


def main():
    print("=" * 60)
    print("Hello Isaac Sim - Basic Physics Simulation")
    print("=" * 60)

    # Create World (simulation manager)
    print("\n[1/5] Creating Isaac Sim World...")
    world = World(
        stage_units_in_meters=1.0,  # 1 USD unit = 1 meter
        physics_dt=1.0/60.0,         # Physics timestep: 60 Hz
        rendering_dt=1.0/60.0        # Rendering timestep: 60 Hz
    )
    print("✓ World created successfully")

    # Add ground plane
    print("\n[2/5] Adding ground plane...")
    ground = world.scene.add(
        GroundPlane(
            prim_path="/World/Ground",
            size=10.0,  # 10m x 10m
            color=np.array([0.5, 0.5, 0.5])  # Gray
        )
    )
    print("✓ Ground plane added at /World/Ground")

    # Add dynamic cube (will fall under gravity)
    print("\n[3/5] Adding dynamic cube...")
    cube = world.scene.add(
        DynamicCuboid(
            prim_path="/World/Cube",
            name="test_cube",
            position=np.array([0.0, 0.0, 2.0]),  # 2m above ground
            size=0.5,  # 0.5m cube
            color=np.array([0.0, 0.5, 1.0])  # Blue
        )
    )
    print("✓ Cube added at /World/Cube (2m above ground)")

    # Reset simulation (initialize physics)
    print("\n[4/5] Initializing simulation...")
    world.reset()
    print("✓ Simulation initialized")

    # Run simulation loop
    print("\n[5/5] Running simulation for 5 seconds...")
    print("-" * 60)

    sim_duration = 5.0  # seconds
    steps = int(sim_duration / world.get_physics_dt())

    start_time = time.time()

    for i in range(steps):
        # Step physics and rendering
        world.step(render=True)

        # Get cube position every 0.5 seconds
        if i % 30 == 0:  # 30 steps = 0.5 seconds at 60 Hz
            position, _ = cube.get_world_pose()
            sim_time = world.current_time
            print(f"[t={sim_time:.2f}s] Cube Z-position: {position[2]:.3f} m")

    wall_time = time.time() - start_time
    rtf = sim_duration / wall_time

    print("-" * 60)
    print("\n✓ Simulation complete!")
    print(f"\nPerformance Metrics:")
    print(f"  Simulation time: {sim_duration:.2f}s")
    print(f"  Wall-clock time: {wall_time:.2f}s")
    print(f"  Real-Time Factor (RTF): {rtf:.2f}x")

    if rtf >= 1.0:
        print(f"  ✓ Real-time achieved (RTF ≥ 1.0)")
    else:
        print(f"  ⚠ Below real-time (RTF < 1.0) - reduce scene complexity")

    # Final cube position
    final_position, _ = cube.get_world_pose()
    print(f"\nFinal cube position: [{final_position[0]:.3f}, {final_position[1]:.3f}, {final_position[2]:.3f}] m")

    if final_position[2] < 0.5:
        print("✓ Cube successfully fell to ground under gravity")
    else:
        print("⚠ Warning: Cube did not reach ground (physics issue?)")

    print("\n" + "=" * 60)
    print("Hello Isaac Sim example completed successfully!")
    print("=" * 60)


if __name__ == "__main__":
    main()
