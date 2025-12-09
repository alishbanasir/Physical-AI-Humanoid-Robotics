#!/usr/bin/env python3
"""
Example 07: PBR Materials
Applies physically-based rendering materials to robot links.

Usage:
    ./python.sh main.py
"""

from pxr import UsdShade, Sdf, Gf
import omni.usd


def create_pbr_material(stage, material_path: str, roughness: float, metallic: float, color: tuple):
    """Create PBR material with specified properties."""
    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, f"{material_path}/Shader")
    shader.CreateIdAttr("OmniPBR")

    # Set material properties
    shader.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness_constant", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(metallic)

    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "out")
    return material


def apply_materials():
    """Apply PBR materials to robot links."""
    print("Applying PBR materials...")

    stage = omni.usd.get_context().get_stage()

    # Create materials
    print("  Creating metallic material...")
    metal_mat = create_pbr_material(stage, "/World/Materials/Metal", roughness=0.2, metallic=1.0, color=(0.8, 0.8, 0.8))

    print("  Creating matte plastic material...")
    plastic_mat = create_pbr_material(stage, "/World/Materials/Plastic", roughness=0.8, metallic=0.0, color=(0.2, 0.2, 0.8))

    print("  Creating rubber material...")
    rubber_mat = create_pbr_material(stage, "/World/Materials/Rubber", roughness=0.9, metallic=0.0, color=(0.1, 0.1, 0.1))

    print("✓ PBR materials created successfully!")
    print("  - Metal: roughness=0.2, metallic=1.0")
    print("  - Plastic: roughness=0.8, metallic=0.0")
    print("  - Rubber: roughness=0.9, metallic=0.0")


if __name__ == "__main__":
    apply_materials()
