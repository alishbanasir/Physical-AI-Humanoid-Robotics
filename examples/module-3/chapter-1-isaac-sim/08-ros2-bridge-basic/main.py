#!/usr/bin/env python3
"""
Example 08: ROS 2 Bridge Basic
Demonstrates Isaac Sim ROS 2 bridge for joint state publishing.

Usage:
    Terminal 1: ./python.sh main.py
    Terminal 2: ros2 topic echo /joint_states
"""

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.extensions import enable_extension
import omni.isaac.ros2_bridge


def setup_ros2_bridge():
    """Configure ROS 2 bridge for joint state publishing."""
    print("Setting up ROS 2 bridge...")

    enable_extension("omni.isaac.ros2_bridge")

    world = World()
    robot = world.scene.add(Articulation(prim_path="/World/Robot", name="robot"))

    # Create joint state publisher
    print("  Creating joint state publisher on /joint_states...")
    joint_pub = omni.isaac.ros2_bridge.create_joint_state_publisher(
        prim_path="/World/Robot",
        topic_name="/joint_states",
        publish_rate=50.0
    )

    # Create clock publisher
    print("  Creating clock publisher on /clock...")
    clock_pub = omni.isaac.ros2_bridge.create_clock_publisher(
        topic_name="/clock",
        publish_rate=100.0
    )

    print("✓ ROS 2 bridge configured successfully!")
    print("\nTopics published:")
    print("  - /joint_states (50 Hz)")
    print("  - /clock (100 Hz)")
    print("\nVerify with: ros2 topic list")

    # Run simulation
    world.reset()
    for i in range(600):
        world.step(render=True)
        if i % 60 == 0:
            print(f"  Simulation time: {world.current_time:.2f}s")


if __name__ == "__main__":
    setup_ros2_bridge()
