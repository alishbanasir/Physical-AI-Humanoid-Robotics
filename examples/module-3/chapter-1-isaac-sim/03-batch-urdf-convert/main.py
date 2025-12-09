#!/usr/bin/env python3
"""
Example 03: Batch URDF to USD Conversion
Converts multiple URDF files to USD format in batch mode.

Usage:
    ./python.sh main.py --input-dir /path/to/urdfs/ --output-dir /path/to/usds/
"""

import argparse
import os
import glob
import time
from omni.isaac.core.utils.extensions import enable_extension


def batch_convert_urdfs(input_dir: str, output_dir: str, config: dict) -> dict:
    """Batch convert all URDF files in directory to USD."""
    enable_extension("omni.importer.urdf")
    from omni.importer.urdf import _urdf

    urdf_files = glob.glob(os.path.join(input_dir, "*.urdf"))
    results = {"success": [], "failed": []}

    print(f"Found {len(urdf_files)} URDF files in {input_dir}")

    for urdf_path in urdf_files:
        filename = os.path.basename(urdf_path)
        usd_filename = filename.replace(".urdf", ".usd")
        usd_path = os.path.join(output_dir, usd_filename)

        print(f"\nConverting: {filename} → {usd_filename}")

        try:
            _urdf.acquire_urdf_interface().import_robot(
                urdf_path, usd_path,
                fix_base=config.get("fix_base", False),
                merge_fixed_joints=config.get("merge_fixed_joints", True),
                self_collision=config.get("self_collision", True),
                create_physics_scene=False  # Don't create physics scene for batch
            )
            results["success"].append(filename)
            print(f"  ✓ Success")
        except Exception as e:
            results["failed"].append((filename, str(e)))
            print(f"  ✗ Failed: {e}")

    return results


def main():
    parser = argparse.ArgumentParser(description="Batch URDF to USD conversion")
    parser.add_argument("--input-dir", type=str, required=True, help="Input directory with URDF files")
    parser.add_argument("--output-dir", type=str, required=True, help="Output directory for USD files")
    args = parser.parse_args()

    # Create output directory
    os.makedirs(args.output_dir, exist_ok=True)

    print("=" * 60)
    print("Batch URDF to USD Conversion")
    print("=" * 60)

    config = {"fix_base": False, "merge_fixed_joints": True, "self_collision": True}

    start_time = time.time()
    results = batch_convert_urdfs(args.input_dir, args.output_dir, config)
    elapsed = time.time() - start_time

    print("\n" + "=" * 60)
    print("Batch Conversion Complete")
    print("=" * 60)
    print(f"Successful: {len(results['success'])}")
    print(f"Failed: {len(results['failed'])}")
    print(f"Total time: {elapsed:.2f}s")

    if results["failed"]:
        print("\nFailed conversions:")
        for filename, error in results["failed"]:
            print(f"  - {filename}: {error}")


if __name__ == "__main__":
    main()
