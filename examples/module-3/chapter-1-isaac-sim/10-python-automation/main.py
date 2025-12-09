#!/usr/bin/env python3
"""
Example 10: Python Automation
Demonstrates advanced automation: parameter sweep, multi-robot spawning, data collection.

Usage:
    ./python.sh main.py --mode sweep
    ./python.sh main.py --mode multi-robot
    ./python.sh main.py --mode data-collection
"""

import argparse
import time
import json
import csv
import numpy as np
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import PhysxSchema
import omni.usd


def parameter_sweep(robot_usd: str):
    """Test different PhysX solver iterations."""
    print("=" * 60)
    print("Parameter Sweep: PhysX Solver Iterations")
    print("=" * 60)

    solver_configs = [4, 8, 16, 24, 32]
    results = []

    for solver_iter in solver_configs:
        print(f"\nTesting solver iterations: {solver_iter}")

        world = World()
        add_reference_to_stage(usd_path=robot_usd, prim_path="/World/Robot")
        robot = world.scene.add(Articulation(prim_path="/World/Robot", name="robot"))

        # Configure solver
        stage = omni.usd.get_context().get_stage()
        physics_scene = stage.GetPrimAtPath("/World/PhysicsScene")
        physx_api = PhysxSchema.PhysxSceneAPI.Apply(physics_scene)
        physx_api.CreateSolverPositionIterationCountAttr(solver_iter)

        world.reset()
        robot.initialize()

        # Measure RTF and stability
        start_time = time.time()
        sim_duration = 5.0
        steps = int(sim_duration / world.get_physics_dt())

        stable = True
        for i in range(steps):
            world.step(render=False)
            pos, _ = robot.get_world_pose()
            if pos[2] < 0.3:
                stable = False
                break

        wall_time = time.time() - start_time
        rtf = sim_duration / wall_time

        results.append({"solver_iter": solver_iter, "rtf": rtf, "stable": stable})
        print(f"  RTF: {rtf:.2f}x, Stable: {stable}")

        world.clear()

    # Save results
    with open("solver_sweep_results.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["solver_iter", "rtf", "stable"])
        writer.writeheader()
        writer.writerows(results)

    print("\n✓ Results saved to solver_sweep_results.csv")


def multi_robot_spawning(robot_usd: str, num_robots: int = 5):
    """Spawn multiple robots in array formation."""
    print("=" * 60)
    print(f"Multi-Robot Spawning: {num_robots} robots")
    print("=" * 60)

    world = World()
    robots = []

    for i in range(num_robots):
        prim_path = f"/World/Robot_{i:02d}"
        position = np.array([i * 2.0, 0.0, 1.0])

        add_reference_to_stage(usd_path=robot_usd, prim_path=prim_path)
        robot = world.scene.add(Articulation(prim_path=prim_path, name=f"robot_{i:02d}", position=position))
        robots.append(robot)
        print(f"  Spawned robot_{i:02d} at position {position}")

    world.reset()
    print(f"\n✓ {len(robots)} robots spawned successfully")

    # Simulate
    for i in range(600):
        world.step(render=True)


def data_collection(robot_usd: str, num_samples: int = 100):
    """Collect trajectory data for ML."""
    print("=" * 60)
    print(f"Data Collection: {num_samples} samples")
    print("=" * 60)

    world = World()
    add_reference_to_stage(usd_path=robot_usd, prim_path="/World/Robot")
    robot = world.scene.add(Articulation(prim_path="/World/Robot", name="robot"))

    world.reset()
    robot.initialize()

    trajectory_data = []

    for step in range(num_samples):
        world.step(render=False)

        # Apply random commands
        random_pos = np.random.uniform(-0.5, 0.5, robot.num_dof)
        robot.set_joint_position_targets(random_pos.tolist())

        # Collect data
        obs = {
            "step": step,
            "time": world.current_time,
            "joint_positions": robot.get_joint_positions().tolist(),
            "joint_velocities": robot.get_joint_velocities().tolist(),
            "base_position": robot.get_world_pose()[0].tolist()
        }
        trajectory_data.append(obs)

        if step % 10 == 0:
            print(f"  Collected {step}/{num_samples} samples...")

    # Save
    with open("trajectory_data.json", "w") as f:
        json.dump(trajectory_data, f, indent=2)

    print(f"\n✓ {len(trajectory_data)} samples saved to trajectory_data.json")


def main():
    parser = argparse.ArgumentParser(description="Python automation examples")
    parser.add_argument("--mode", type=str, required=True, choices=["sweep", "multi-robot", "data-collection"])
    parser.add_argument("--robot", type=str, default="/path/to/humanoid.usd", help="Robot USD file")
    parser.add_argument("--num-robots", type=int, default=5, help="Number of robots (multi-robot mode)")
    parser.add_argument("--num-samples", type=int, default=100, help="Number of samples (data-collection mode)")

    args = parser.parse_args()

    if args.mode == "sweep":
        parameter_sweep(args.robot)
    elif args.mode == "multi-robot":
        multi_robot_spawning(args.robot, args.num_robots)
    elif args.mode == "data-collection":
        data_collection(args.robot, args.num_samples)


if __name__ == "__main__":
    main()
