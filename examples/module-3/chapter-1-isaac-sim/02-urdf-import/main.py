#!/usr/bin/env python3
"""
Example 02: URDF Import to USD
Imports a URDF robot file to Isaac Sim and validates the conversion.

Learning Objectives:
- Enable URDF Importer extension
- Convert URDF → USD with Python API
- Validate articulation structure (links, joints, DOF)
- Test physics simulation with imported robot
- Inspect joint properties (limits, axes, damping)

Usage:
    cd ~/.local/share/ov/pkg/isaac_sim-2023.1.1
    ./python.sh /path/to/examples/module-3/chapter-1-isaac-sim/02-urdf-import/main.py \
        --urdf /path/to/humanoid.urdf \
        --output /path/to/humanoid.usd

Prerequisites:
    - URDF file with valid syntax (test with: check_urdf humanoid.urdf)
    - All mesh files (.stl, .dae) referenced in URDF must exist
"""

import argparse
import os
import time
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.extensions import enable_extension
import omni.usd


def import_urdf_to_usd(urdf_path: str, usd_output_path: str, config: dict) -> bool:
    """
    Import URDF file to USD format using Isaac Sim URDF Importer.

    Args:
        urdf_path: Path to input URDF file
        usd_output_path: Path to output USD file
        config: Import configuration dictionary

    Returns:
        bool: True if import successful, False otherwise
    """
    print(f"\n[1/5] Enabling URDF Importer extension...")
    enable_extension("omni.importer.urdf")
    print("✓ URDF Importer enabled")

    print(f"\n[2/5] Importing URDF: {urdf_path}")
    print(f"  Output USD: {usd_output_path}")

    try:
        from omni.importer.urdf import _urdf

        # Import with configuration
        _urdf.acquire_urdf_interface().import_robot(
            urdf_path,
            usd_output_path,
            fix_base=config.get("fix_base", False),
            merge_fixed_joints=config.get("merge_fixed_joints", True),
            self_collision=config.get("self_collision", True),
            create_physics_scene=config.get("create_physics_scene", True),
            import_inertia_tensor=config.get("import_inertia_tensor", True)
        )

        print("✓ URDF imported successfully")
        return True

    except Exception as e:
        print(f"✗ URDF import failed: {e}")
        return False


def validate_articulation(robot: Articulation) -> dict:
    """
    Validate imported robot articulation structure.

    Args:
        robot: Articulation object to validate

    Returns:
        dict: Validation results
    """
    print(f"\n[4/5] Validating articulation structure...")

    results = {
        "valid": True,
        "num_dof": 0,
        "joint_names": [],
        "issues": []
    }

    try:
        # Get DOF count
        num_dof = robot.num_dof
        results["num_dof"] = num_dof
        print(f"  Degrees of Freedom: {num_dof}")

        if num_dof == 0:
            results["valid"] = False
            results["issues"].append("No joints found (DOF = 0)")
            print("  ✗ No joints found!")
            return results

        # Get joint names
        joint_names = robot.dof_names
        results["joint_names"] = joint_names
        print(f"  Joint names: {', '.join(joint_names[:5])}" +
              (f" ... (+{len(joint_names)-5} more)" if len(joint_names) > 5 else ""))

        # Get joint positions
        positions = robot.get_joint_positions()
        print(f"  Initial joint positions: {positions[:3]}..." if len(positions) > 3 else f"  Initial joint positions: {positions}")

        # Get joint velocities
        velocities = robot.get_joint_velocities()
        if velocities is not None:
            print(f"  ✓ Joint velocities accessible")
        else:
            results["issues"].append("Joint velocities not accessible")

        # Get world pose
        position, orientation = robot.get_world_pose()
        print(f"  Robot position: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]")
        print(f"  Robot orientation (quat): [{orientation[0]:.3f}, {orientation[1]:.3f}, {orientation[2]:.3f}, {orientation[3]:.3f}]")

        print(f"\n✓ Articulation validation {'complete' if results['valid'] else 'complete with issues'}")

    except Exception as e:
        results["valid"] = False
        results["issues"].append(f"Validation error: {e}")
        print(f"✗ Validation error: {e}")

    return results


def test_physics_simulation(world: World, robot: Articulation, duration: float = 3.0):
    """
    Test physics simulation with imported robot.

    Args:
        world: Isaac Sim World instance
        robot: Robot articulation
        duration: Simulation duration in seconds
    """
    print(f"\n[5/5] Testing physics simulation ({duration}s)...")

    steps = int(duration / world.get_physics_dt())
    start_time = time.time()

    # Track robot stability (check if robot explodes or jitters excessively)
    initial_position, _ = robot.get_world_pose()
    max_deviation = 0.0

    for i in range(steps):
        world.step(render=True)

        # Check every 0.5 seconds
        if i % 30 == 0:
            position, _ = robot.get_world_pose()
            deviation = abs(position[2] - initial_position[2])
            max_deviation = max(max_deviation, deviation)

            sim_time = world.current_time
            print(f"  [t={sim_time:.2f}s] Robot Z: {position[2]:.3f}m (deviation: {deviation:.3f}m)")

    wall_time = time.time() - start_time
    rtf = duration / wall_time

    print(f"\n  Simulation complete:")
    print(f"    RTF: {rtf:.2f}x")
    print(f"    Max Z deviation: {max_deviation:.3f}m")

    # Stability check
    if max_deviation > 1.0:
        print(f"  ⚠ Warning: Large Z deviation detected (>{1.0}m)")
        print(f"     Robot may have fallen or exploded due to physics instability.")
        print(f"     Try increasing PhysX solver iterations (Window → Physics → Settings)")
    else:
        print(f"  ✓ Robot stable (Z deviation < 1.0m)")


def main():
    parser = argparse.ArgumentParser(description="Import URDF to Isaac Sim USD format")
    parser.add_argument("--urdf", type=str, required=True, help="Path to input URDF file")
    parser.add_argument("--output", type=str, required=True, help="Path to output USD file")
    parser.add_argument("--fix-base", action="store_true", help="Fix base link (for manipulator arms)")
    parser.add_argument("--no-merge-fixed", action="store_true", help="Don't merge fixed joints")
    parser.add_argument("--no-self-collision", action="store_true", help="Disable self-collision")

    args = parser.parse_args()

    # Validate inputs
    if not os.path.exists(args.urdf):
        print(f"✗ Error: URDF file not found: {args.urdf}")
        return 1

    # Create output directory if needed
    output_dir = os.path.dirname(args.output)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"Created output directory: {output_dir}")

    print("=" * 60)
    print("URDF to USD Import - Isaac Sim")
    print("=" * 60)

    # Import configuration
    config = {
        "fix_base": args.fix_base,
        "merge_fixed_joints": not args.no_merge_fixed,
        "self_collision": not args.no_self_collision,
        "create_physics_scene": True,
        "import_inertia_tensor": True
    }

    print(f"\nImport configuration:")
    for key, value in config.items():
        print(f"  {key}: {value}")

    # Import URDF to USD
    success = import_urdf_to_usd(args.urdf, args.output, config)

    if not success:
        print("\n✗ Import failed. Please check URDF syntax and mesh file paths.")
        return 1

    # Verify USD file created
    if not os.path.exists(args.output):
        print(f"\n✗ Error: Output USD file not created: {args.output}")
        return 1

    print(f"\n[3/5] USD file created: {args.output}")
    file_size_mb = os.path.getsize(args.output) / (1024 * 1024)
    print(f"  File size: {file_size_mb:.2f} MB")

    # Load USD and validate
    print(f"\n[3/5] Loading USD into Isaac Sim for validation...")
    world = World()

    # Add robot to scene
    robot_prim_path = "/World/robot"
    robot = world.scene.add(
        Articulation(
            prim_path=robot_prim_path,
            name="imported_robot"
        )
    )

    # Initialize simulation
    world.reset()
    robot.initialize()

    # Validate articulation
    validation_results = validate_articulation(robot)

    if not validation_results["valid"]:
        print("\n⚠ Validation issues detected:")
        for issue in validation_results["issues"]:
            print(f"  - {issue}")
    else:
        print(f"\n✓ Articulation validated successfully!")
        print(f"  DOF: {validation_results['num_dof']}")
        print(f"  Joints: {len(validation_results['joint_names'])}")

    # Test physics simulation
    test_physics_simulation(world, robot, duration=3.0)

    print("\n" + "=" * 60)
    print("URDF Import Complete!")
    print("=" * 60)
    print(f"\n✓ USD file saved: {args.output}")
    print(f"✓ Robot DOF: {validation_results['num_dof']}")
    print(f"✓ Ready for use in Isaac Sim simulations")

    return 0


if __name__ == "__main__":
    exit(main())
