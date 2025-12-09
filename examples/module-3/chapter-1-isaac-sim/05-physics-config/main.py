#!/usr/bin/env python3
"""
Example 05: PhysX Configuration
Demonstrates PhysX parameter tuning for humanoid robot stability.

Usage:
    ./python.sh main.py --solver-iterations 16
"""

import argparse
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from pxr import PhysxSchema
import omni.usd


def configure_physx(solver_iterations: int = 16):
    """Configure PhysX scene parameters."""
    print(f"Configuring PhysX with {solver_iterations} solver iterations...")

    world = World()
    robot = world.scene.add(Articulation(prim_path="/World/Robot", name="robot"))

    # Get PhysX scene
    stage = omni.usd.get_context().get_stage()
    physics_scene_path = "/World/PhysicsScene"
    physics_scene = stage.GetPrimAtPath(physics_scene_path)

    # Apply PhysX configuration
    physx_api = PhysxSchema.PhysxSceneAPI.Apply(physics_scene)
    physx_api.CreateSolverPositionIterationCountAttr(solver_iterations)
    physx_api.CreateEnableCCDAttr(True)
    physx_api.CreateEnableStabilizationAttr(True)

    print(f"✓ PhysX configured: {solver_iterations} iterations, CCD enabled")

    # Test simulation
    world.reset()
    for i in range(600):
        world.step(render=False)
        if i % 60 == 0:
            pos, _ = robot.get_world_pose()
            print(f"  t={i/60:.1f}s: Z={pos[2]:.3f}m")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--solver-iterations", type=int, default=16)
    args = parser.parse_args()
    configure_physx(args.solver_iterations)


if __name__ == "__main__":
    main()
