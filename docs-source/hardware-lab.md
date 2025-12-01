---
title: "Hardware Lab Options"
description: Recommended robot platforms and hardware setups for hands-on learning.
tags: [hardware, robots, platforms, lab-setup]
---

# Hardware Lab Options

This section recommends hardware platforms for running hands-on experiments. All platforms have ROS 2 support and integrate with the simulation workflows covered in this textbook.

## Budget Tiers

### Tier 1: Low-Cost Simulators (No Hardware Required)
- **Cost**: Free–$200
- **Best for**: Beginner courses, prototyping

| Platform | Features | Pros | Cons |
|----------|----------|------|------|
| Gazebo (Free) | Open-source physics sim, URDF support | Free, well-documented | Limited realism |
| V-REP / CoppeliaSim | Physics-based simulator, 4+ DOF robots | Flexible, good physics | Proprietary |
| Isaac Sim (Free Tier) | NVIDIA's platform, photorealistic | Photorealistic rendering | Requires GPU |

### Tier 2: Affordable Real Robots ($1K–$5K)
- **Cost**: $1,000–$5,000
- **Best for**: Lab courses, team projects

| Platform | DOF | ROS 2 Support | Use Case |
|----------|-----|---------------|----------|
| Dobot Magician | 4 | Yes | Pick-and-place, desktop |
| uFactory uArm | 4 | Yes | Lightweight, low cost |
| Ufactory xArm 5 | 5 | Yes | Collaborative, safer |
| TurtleBot 4 | Differential drive | Yes | Mobile manipulation, education |

### Tier 3: Research-Grade Robots ($10K–$50K+)
- **Cost**: $10,000–$100,000+
- **Best for**: Advanced research, real-world deployment

| Platform | DOF | Features | ROS 2 Support |
|----------|-----|----------|---------------|
| Boston Dynamics Spot | Mobile | Quadruped, rugged | Community support |
| UR Cobot (UR10e) | 6 | Industrial-grade, safe | Yes (via community) |
| KUKA LBR iiwa | 7 | Lightweight, sensitive | Yes |
| Franka Emika Panda | 7 | Torque-controlled, safe | Yes |

### Tier 4: Humanoid Robots ($50K–$500K+)
- **Cost**: $50,000–$500,000+
- **Best for**: Advanced capstone projects, cutting-edge research

| Platform | Features | Cost Estimate |
|----------|----------|---------------|
| Boston Dynamics Atlas | Full-body humanoid, advanced locomotion | $300K+ |
| Tesla Optimus (Early Access) | Humanoid, designed for manufacturing | TBD |
| Figure AI Figure 01 | Humanoid, bipedal locomotion | $150K+  |
| iCub | Research humanoid, open-source | $500K+ (assembly) |

## Recommended Lab Setup (Budget: $5K–$10K)

For a university course serving 20–30 students, we recommend:

1. **Simulation Lab** (all students)
   - Gazebo + Isaac Sim (free or low-cost licenses)
   - Cloud compute (AWS, Google Cloud, Azure credits)
   - No hardware required for initial modules

2. **Hardware Lab** (3–4 teams, final capstone)
   - 1x Franka Emika Panda (or UR10e) — $15K–$20K
   - 1x Mobile base (TurtleBot 4 or similar) — $2K–$4K
   - Cameras, grippers, sensors — $3K–$5K
   - **Total**: ~$25K–$30K for full setup

3. **Serverless Backend** (VLA agents, LLM inference)
   - OpenAI API + Anthropic Claude API
   - Neon Postgres database (free tier available)
   - Qdrant vector DB (free tier available)
   - **Monthly cost**: $100–$500 (depending on usage)

## Setting Up a Robot

### Quick Start: Desktop Arm (Dobot Magician)

1. **Hardware Assembly**: Follow vendor documentation (~30 min)
2. **Install ROS 2 Driver**: Clone the driver repo and build
   ```bash
   git clone https://github.com/dobot-ros2/dobot_magician_ros2.git
   cd dobot_magician_ros2
   colcon build
   ```
3. **Bring-up**: Launch the driver node
   ```bash
   ros2 launch dobot_magician_bringup bringup.launch.py
   ```
4. **Verify**: Check that `/joint_states` and `/cmd_vel` topics work
   ```bash
   ros2 topic list
   ros2 topic echo /joint_states
   ```

### Advanced: Setting Up a Franka Emika Panda

1. **System Requirements**:
   - Real-time kernel (Linux, Preempt-RT patch recommended)
   - Franka Control Interface (FCI) enabled
   - ROS 2 + `frankapy` or `franka_ros2` installed

2. **Calibration**: Perform force/torque sensor calibration (vendor docs)

3. **Safety**: Test emergency stop, joint limits, collision detection

4. **Testing**: Run a simple motion command
   ```python
   from franka_python_interface import ArmInterface
   arm = ArmInterface()
   arm.move_to_joint_angles({'panda_joint1': 0.0, ...})
   ```

## Safety Guidelines

When working with physical robots:

1. **Personal Safety**
   - Wear safety glasses
   - Keep hands clear of moving joints
   - Never reach into the workspace without stopping the robot

2. **Robot Safety**
   - Test in simulation first
   - Use low speed/force limits for initial tests
   - Monitor torque/force sensors for anomalies
   - Implement emergency stop (e-stop) hardware

3. **Software Safety**
   - Validate all commands before execution
   - Use timeout mechanisms to stop hanging processes
   - Log all robot actions for debugging
   - Test failure modes (sensor failure, communication loss)

4. **Lab Setup**
   - Clear workspace of obstacles
   - Install protective barriers around robot
   - Maintain a logbook of incidents
   - Have an emergency protocol document

## Further Reading

- [ROS 2 Hardware Abstraction](https://docs.ros.org/en/iron/Concepts/About-ROS-2-Hardware-Acceleration.html)
- [Robot Safety Standards (ISO/TS 15066)](https://www.iso.org/standard/62996.html)
- [Collaborative Robot Best Practices](https://www.roboticstomorrow.com/article/2022/collaborative-robots/)

---

**Related:** [Module 1: ROS 2 Setup](./module-1-ros2.md)
