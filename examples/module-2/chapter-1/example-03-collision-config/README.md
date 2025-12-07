# Example 03: Collision Configuration

## Description
Demonstrates configuring collision geometries and surface properties (friction, contact stiffness, damping) for realistic object interaction.

## Key Concepts
- Friction coefficients (mu1, mu2)
- Contact stiffness (kp) and damping (kd)
- Collision geometry optimization

## Usage
```bash
# Create obstacle SDF model with collision properties
# Spawn humanoid + obstacle
# Test collision behavior
```

## Learning Outcomes
- Configure `<mu1>`, `<mu2>` friction parameters
- Tune `<kp>` (stiffness) and `<kd>` (damping) for stable contacts
- Prevent object penetration and jitter
