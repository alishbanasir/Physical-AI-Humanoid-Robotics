#!/usr/bin/env python3
"""
Example 04: Custom USD Scene Creation
Creates a custom simulation environment with ground, obstacles, and robot.

Usage:
    ./python.sh main.py --robot /path/to/humanoid.usd --output scene.usd
"""

import argparse
from omni.isaac.core import World
from omni.isaac.core.objects import GroundPlane, DynamicCuboid, DynamicCylinder
from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.usd
import numpy as np


def create_custom_scene(robot_usd_path: str, output_path: str):
    """Create custom scene with robot, ground, and obstacles."""
    print("Creating custom scene...")

    world = World(stage_units_in_meters=1.0)

    # Add ground plane
    print("  Adding ground plane...")
    ground = world.scene.add(
        GroundPlane(prim_path="/World/Ground", size=20.0, color=np.array([0.5, 0.5, 0.5]))
    )

    # Add robot
    print(f"  Adding robot from: {robot_usd_path}")
    add_reference_to_stage(usd_path=robot_usd_path, prim_path="/World/Robot")

    # Add obstacles
    print("  Adding obstacles...")
    obstacles = [
        DynamicCuboid(prim_path="/World/Obstacles/Box_01", position=np.array([2.0, 0.0, 0.5]), size=1.0, color=np.array([1.0, 0.0, 0.0])),
        DynamicCuboid(prim_path="/World/Obstacles/Box_02", position=np.array([-2.0, 1.0, 0.5]), size=0.8, color=np.array([0.0, 1.0, 0.0])),
        DynamicCylinder(prim_path="/World/Obstacles/Cylinder_01", position=np.array([0.0, 3.0, 1.0]), radius=0.3, height=2.0, color=np.array([0.0, 0.0, 1.0]))
    ]

    for obs in obstacles:
        world.scene.add(obs)

    # Save scene
    print(f"  Saving scene to: {output_path}")
    stage = omni.usd.get_context().get_stage()
    stage.Export(output_path)

    print("✓ Custom scene created successfully!")


def main():
    parser = argparse.ArgumentParser(description="Create custom USD scene")
    parser.add_argument("--robot", type=str, required=True, help="Robot USD file path")
    parser.add_argument("--output", type=str, default="custom_scene.usd", help="Output scene file")
    args = parser.parse_args()

    create_custom_scene(args.robot, args.output)


if __name__ == "__main__":
    main()
