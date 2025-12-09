#!/usr/bin/env python3
"""
Example 09: ROS 2 Control
Subscribes to ROS 2 joint commands and applies them to Isaac Sim robot.

Usage:
    Terminal 1: ./python.sh main.py
    Terminal 2: ros2 topic pub /joint_commands sensor_msgs/msg/JointState ...
"""

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.extensions import enable_extension
import omni.isaac.ros2_bridge


def setup_ros2_control():
    """Configure ROS 2 subscriber for joint control."""
    print("Setting up ROS 2 control...")

    enable_extension("omni.isaac.ros2_bridge")

    world = World()
    robot = world.scene.add(Articulation(prim_path="/World/Robot", name="robot"))

    # Create joint command subscriber
    print("  Creating joint command subscriber on /joint_commands...")
    joint_cmd_sub = omni.isaac.ros2_bridge.create_joint_command_subscriber(
        prim_path="/World/Robot",
        topic_name="/joint_commands",
        command_type="position"
    )

    print("✓ ROS 2 control configured successfully!")
    print("\nTopics subscribed:")
    print("  - /joint_commands (position control)")
    print("\nSend commands with:")
    print("  ros2 topic pub /joint_commands sensor_msgs/msg/JointState ...")

    # Run simulation
    world.reset()
    robot.initialize()

    print(f"\nAvailable joints ({robot.num_dof}):")
    for name in robot.dof_names[:5]:
        print(f"  - {name}")
    if robot.num_dof > 5:
        print(f"  ... (+{robot.num_dof-5} more)")

    for i in range(6000):  # Run for 100 seconds
        world.step(render=True)


if __name__ == "__main__":
    setup_ros2_control()
