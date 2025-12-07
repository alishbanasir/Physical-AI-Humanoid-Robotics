# Example 05: External Force Application

## Description
Demonstrates applying external forces to robot links using Gazebo services and ROS 2 to test stability and validate control algorithms.

## Key Concepts
- `/gazebo/apply_body_wrench` service
- Force and torque vectors
- Impulse vs sustained forces
- Robot stability testing

## Usage
```bash
# Launch humanoid in Gazebo
# Apply 50N push force in +X direction
ros2 service call /gazebo/apply_body_wrench gazebo_msgs/srv/ApplyBodyWrench \
  "body_name: 'simple_humanoid::base_link'
   wrench: {force: {x: 50.0, y: 0.0, z: 0.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}
   duration: {sec: 1, nanosec: 0}"
```

## Test Scenarios
- **Push Test**: Apply lateral forces to test balance
- **Torque Test**: Apply rotational torques to test stability
- **Wind Simulation**: Apply sustained forces to simulate environmental disturbances
