---
title: "Capstone Project"
description: 12-week team project integrating ROS 2, simulation, perception, and VLA agents for a real-world robotic task.
tags: [capstone, project, teams, evaluation]
---

# Capstone Project: Build Your Own Physical AI System

The capstone project is the culmination of all four modules. In teams of 2–4, you'll design and deploy a complete robotic system that integrates:
- **ROS 2 architecture** (Module 1)
- **Simulation and digital twins** (Module 2)
- **Perception pipelines** (Module 3)
- **Vision & Language Agents** (Module 4)

## Project Timeline

- **Week 11**: Final project selection, team formation, and project plan
- **Week 12**: Project development, integration, testing, and presentation

## Learning Outcomes

By completing the capstone, you will:
- **Design** a modular robotic system from scratch
- **Implement** multi-node ROS 2 architectures with proper communication
- **Integrate** simulation, perception, and control in a closed loop
- **Validate** your system in simulation and on hardware (if available)
- **Document** your work for reproducibility

## Project Ideas

### Idea 1: Robotic Pick-and-Place with VLA
**Task**: Train a robot to pick objects specified by natural language.

**System Components**:
- Camera mounted on robot arm
- VLM-based task decomposition (Claude Vision / GPT-4V)
- Grasp detection network (trained on synthetic data from Isaac Sim)
- ROS 2 motion planning (MoveIt2)
- Closed-loop feedback control

**Success Metrics**:
- Pick-and-place success rate: > 80% in simulation
- VLA correctly interprets 10+ different instructions
- System works on real hardware (if available)

### Idea 2: Autonomous Table Cleanup
**Task**: Robot identifies clutter on a table and tidies it up using natural language commands.

**System Components**:
- Semantic segmentation (identify object types)
- Trajectory planning (avoid collisions)
- Gripper control (adjust grip force based on object)
- VLA agent (decide object placement zones)
- Real-time perception loop

**Success Metrics**:
- Detect and classify 10+ object types
- Execute 5+ consecutive cleanup actions
- Achieve > 90% collision-free trajectories

### Idea 3: Mobile Manipulation Task (Advanced)
**Task**: Deploy a mobile robot (TurtleBot + arm) to fetch and deliver objects.

**System Components**:
- SLAM for localization (mapping unknown environments)
- Path planning (navigate around obstacles)
- Manipulation pipeline (pick object at location A, place at B)
- VLA for task understanding (e.g., "bring me the red cup from the kitchen")
- Multi-node ROS 2 system

**Success Metrics**:
- Navigate > 50 meters autonomously
- Complete 3+ multi-step tasks
- Robustness to dynamic obstacles

### Idea 4: Sim-to-Real Transfer Study
**Task**: Evaluate domain randomization techniques for sim-to-real transfer.

**System Components**:
- Train a control policy in Isaac Sim
- Vary simulation parameters (friction, mass, lighting)
- Test on real hardware
- Measure success rate vs. simulation fidelity

**Success Metrics**:
- Sim success rate: > 95%
- Real-world success rate: > 70%
- Document lessons learned

## Project Requirements

### Code Organization
```
capstone-<team-name>/
├── ros2_ws/
│   ├── src/
│   │   ├── robot_description/
│   │   ├── perception/
│   │   ├── manipulation/
│   │   └── vla_agent/
│   └── build/
├── simulation/
│   ├── isaac_sim_configs/
│   └── gazebo_models/
├── data/
│   ├── synthetic_dataset/
│   └── real_world_test/
├── tests/
│   ├── unit_tests/
│   └── integration_tests/
├── docs/
│   ├── system_architecture.md
│   ├── setup_instructions.md
│   └── results.md
└── README.md
```

### Deliverables

1. **System Architecture Document** (2–3 pages)
   - Problem statement
   - High-level design (block diagram)
   - Component responsibilities
   - Data flow

2. **Code Repository**
   - Well-structured, documented code
   - Unit tests for each module
   - Integration tests for full system
   - README with setup and run instructions

3. **Simulation Validation**
   - Test cases in Gazebo or Isaac Sim
   - Proof that system works in simulation (video)
   - Success metrics and performance analysis

4. **Hardware Testing Report** (if available)
   - Setup photos
   - Real-world test videos
   - Challenges encountered and solutions
   - Comparison: simulation vs. real world

5. **Final Presentation** (12–15 minutes)
   - Problem and motivation
   - System overview (block diagrams)
   - Demo (video or live)
   - Results and lessons learned
   - Future work

## Evaluation Rubric

| Criterion | Excellent (90–100) | Good (80–89) | Fair (70–79) | Poor (< 70) |
|-----------|-------------------|--------------|--------------|------------|
| **System Design** | Clear, modular, scalable | Well-designed with minor issues | Design works but could be cleaner | Lacks structure, unclear |
| **Code Quality** | Clean, well-documented, tested | Mostly clean, some tests | Works but needs comments | Unclear, untested |
| **ROS 2 Integration** | Proper node architecture, clean IPC | Good use of topics/services | Basic ROS 2 usage | Limited/no ROS 2 use |
| **Simulation** | Validates all components, results reproducible | Good simulation, minor gaps | Basic simulation validation | Little/no simulation |
| **VLA Agent** | Sophisticated prompt engineering, closed-loop control | Good agent, basic feedback | Simple agent, single pass | No VLA or non-functional |
| **Results** | Exceeds goals, thorough analysis | Meets goals, good analysis | Partially achieves goals | Goals not met |
| **Presentation** | Engaging, clear, confident | Well-organized, clear | Somewhat unclear in places | Disorganized, hard to follow |

## Team Roles (Suggested)

- **Tech Lead**: System architecture, ROS 2 design, integration
- **Simulation Engineer**: Isaac Sim/Gazebo setup, synthetic data generation
- **Perception Engineer**: VLM integration, detection/segmentation pipelines
- **Robotics Engineer**: Hardware setup, motion planning, safety
- **Documentation Lead**: README, architecture docs, project reports

Note: Smaller teams can combine roles.

## Submission Checklist

- [ ] GitHub repository with all code
- [ ] README with quick-start instructions
- [ ] System architecture document
- [ ] Unit tests and integration tests (> 80% coverage)
- [ ] Simulation demo video (< 5 min)
- [ ] Hardware testing report (if applicable)
- [ ] Final presentation slides
- [ ] All team members' contributions documented (e.g., via git commits)

## Timeline

- **Day 1 (Week 11)**: Form teams, select project idea
- **Day 2–5**: Design phase, create architecture document
- **Day 6–8**: Implement and test components in simulation
- **Day 9–11**: Integration, debugging, hardware testing (if available)
- **Day 12**: Final polish, presentation prep
- **Presentation Day**: Demo and presentation (20 min per team)

## Resources

- **Starter Template**: https://github.com/physical-ai-textbook/capstone-starter
- **Simulation Environments**: Isaac Sim, Gazebo, PyBullet
- **ROS 2 Tools**: MoveIt2 (motion planning), Nav2 (navigation), RViz (visualization)
- **VLM APIs**: Claude Vision, GPT-4V (sign up for free trials)
- **Hardware**: See [Hardware Lab](./hardware-lab.md)

## Support

- **Office Hours**: 2 hours per week for questions and debugging
- **Discussion Forum**: Post questions and share progress
- **Peer Review**: Teams review each other's code mid-project
- **Guest Speakers**: Industry experts may present on specific topics

---

**Good luck on your capstone project!**

Questions? Check the [FAQ](#faq) or post in the course forum.
