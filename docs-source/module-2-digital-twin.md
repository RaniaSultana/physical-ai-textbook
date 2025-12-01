---
title: "Module 2: Digital Twin & Simulation"
description: Build virtual replicas of robots and run physics-based simulations for behavior testing.
tags: [simulation, digital-twin, physics, gazebo]
---

# Module 2: Digital Twin & Simulation

A **digital twin** is a virtual replica of a physical system. By simulating a robot before deploying to hardware, you can:
- Test algorithms safely
- Explore edge cases
- Reduce development time and hardware damage

In this module, we'll use **Gazebo** (free, open-source) and **NVIDIA Isaac Sim** (proprietary, next module) to build and test digital twins.

## Learning Objectives

- Understand the role of digital twins in robotics development
- Create URDF (Unified Robot Description Format) models
- Build and run simulations in Gazebo
- Apply physics simulation to test control algorithms
- Understand sim-to-real transfer challenges

## Key Concepts

### URDF (Unified Robot Description Format)
URDF is an XML-based format that describes:
- **Links** (rigid bodies, geometry, inertia)
- **Joints** (connections between links, DOF)
- **Sensors** (cameras, LiDAR, IMUs)
- **Actuators** (motors, grippers)

### Gazebo
Gazebo is a physics simulator that renders the URDF model and simulates dynamics, collisions, and sensor behavior.

### Sim-to-Real Transfer
The challenge of moving a trained policy from simulation to real hardware. Techniques:
- **Domain randomization** (vary simulation parameters)
- **Physics calibration** (tune simulation to match real robot)

## Reading

- [URDF Documentation](http://wiki.ros.org/urdf)
- [Gazebo Official Tutorials](https://gazebosim.org/docs)
- [Sim-to-Real Transfer: A Survey](https://arxiv.org/abs/2102.13435)

## Hands-on Exercises

### Exercise 2.1: Write a Simple URDF
Create a URDF for a two-link arm with:
- Base link (fixed to ground)
- Link 1 (0.5m long, 1 kg)
- Link 2 (0.3m long, 0.5 kg)
- Revolute joints at the base and elbow

### Exercise 2.2: Load URDF in Gazebo
Load your URDF from Exercise 2.1 in Gazebo and visualize it.

### Exercise 2.3: Apply a Force
Write a ROS 2 node that applies a force to Link 2 and observe the motion in Gazebo.

### Exercise 2.4: Domain Randomization
Modify the simulation to randomize mass and friction values. Train a simple controller and observe robustness.

## Code Example: Simple URDF (Two-Link Arm)

```xml
<?xml version="1.0"?>
<robot name="simple_arm">
  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>

  <!-- Link 1 -->
  <link name="link1">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.05 0.05 0.5"/>
      </geometry>
    </visual>
  </link>

  <!-- Link 2 -->
  <link name="link2">
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.04 0.04 0.3"/>
      </geometry>
    </visual>
  </link>

  <!-- Joint 1 -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="3.14159" effort="10" velocity="1"/>
  </joint>

  <!-- Joint 2 -->
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="3.14159" effort="5" velocity="1"/>
  </joint>
</robot>
```

---

**Next:** [Module 3: NVIDIA Isaac & Perception](./module-3-isaac.md)
