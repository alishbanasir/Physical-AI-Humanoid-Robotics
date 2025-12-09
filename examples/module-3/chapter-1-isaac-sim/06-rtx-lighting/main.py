#!/usr/bin/env python3
"""
Example 06: RTX Lighting Setup
Creates photorealistic lighting with HDR dome light and area lights.

Usage:
    ./python.sh main.py
"""

from omni.isaac.core import World
from pxr import UsdLux, UsdGeom, Gf
import omni.usd


def setup_rtx_lighting():
    """Configure RTX lighting with HDR dome and area lights."""
    print("Setting up RTX lighting...")

    world = World()
    stage = omni.usd.get_context().get_stage()

    # Create HDR dome light
    print("  Creating HDR dome light...")
    dome_light = UsdLux.DomeLight.Define(stage, "/World/Lights/DomeLight")
    dome_light.CreateIntensityAttr(1000.0)
    dome_light.CreateTextureFormatAttr("latlong")

    # Create key light (main light)
    print("  Creating key light...")
    key_light = UsdLux.RectLight.Define(stage, "/World/Lights/KeyLight")
    key_light.CreateIntensityAttr(10000.0)
    key_light.CreateWidthAttr(2.0)
    key_light.CreateHeightAttr(2.0)

    xform = UsdGeom.Xformable(key_light)
    xform.AddTranslateOp().Set(Gf.Vec3d(3, 3, 3))
    xform.AddRotateXYZOp().Set(Gf.Vec3d(-45, -45, 0))

    # Create fill light
    print("  Creating fill light...")
    fill_light = UsdLux.RectLight.Define(stage, "/World/Lights/FillLight")
    fill_light.CreateIntensityAttr(5000.0)
    fill_light.CreateWidthAttr(2.0)
    fill_light.CreateHeightAttr(2.0)

    xform = UsdGeom.Xformable(fill_light)
    xform.AddTranslateOp().Set(Gf.Vec3d(-3, 3, 2))
    xform.AddRotateXYZOp().Set(Gf.Vec3d(-30, 45, 0))

    print("✓ RTX lighting configured successfully!")
    print("  - HDR dome light: 1000 intensity")
    print("  - Key light: 10000 intensity @ (3,3,3)")
    print("  - Fill light: 5000 intensity @ (-3,3,2)")


if __name__ == "__main__":
    setup_rtx_lighting()
