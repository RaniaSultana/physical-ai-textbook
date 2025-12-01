---
title: "Module 4: Vision & Language Agents (VLA)"
description: Leverage multimodal foundation models for robot instruction following and task learning.
tags: [vla, language-models, multimodal, instruction-following, agents]
---

# Module 4: Vision & Language Agents (VLA)

Vision Language Models (VLM) like GPT-4V and Claude Vision can understand images and text. By combining VLMs with robot control, we create **Vision & Language Agents (VLA)** that:
- Understand natural language instructions
- Perceive the environment via cameras
- Generate robot actions (grasping, moving, pushing)

In this module, we'll build agents that translate human instructions into robot behaviors.

## Learning Objectives

- Understand Vision Language Models (VLM) and their capabilities
- Build a prompt-based agent for task decomposition
- Integrate VLM with robot perception and control
- Design reward functions for reinforcement learning from human feedback (RLHF)
- Deploy agents on real robots with safety constraints

## Key Concepts

### Vision Language Models (VLM)
Models like GPT-4V and Claude Vision can:
- Describe scenes in natural language
- Answer questions about images
- Reason about spatial relationships
- Suggest actions for task completion

### Multimodal Understanding
A VLA combines:
- **Vision** (camera input)
- **Language** (natural language instruction)
- **Motor control** (robot actuators)

### Prompt Engineering for Robotics
Effective prompts structure the task:
```
"You are a robot arm. The scene shows a table with a cup and a bottle.
The goal is: 'Place the cup on the shelf.'
Plan a sequence of actions to accomplish this goal."
```

### Safety in Physical AI
When controlling physical systems, ensure:
- **Action validation** (check joint limits, collisions)
- **Fallback strategies** (stop if uncertain)
- **Operator override** (human can pause/cancel)

## Reading

- [Vision Language Models for Robotics](https://arxiv.org/abs/2212.07981)
- [Learning from Human Feedback](https://arxiv.org/abs/2203.02155)
- [VLA: A Large Vision Language Model for Robotics](https://arxiv.org/abs/2310.09197)
- [Claude Vision API Documentation](https://docs.anthropic.com/en/docs/vision/vision-getting-started)
- [GPT-4V API Documentation](https://platform.openai.com/docs/guides/vision)

## Hands-on Exercises

### Exercise 4.1: VLM Image Understanding
Use Claude Vision or GPT-4V to describe a robot scene. Prompt: "Describe this scene and suggest 3 actions the robot could take."

### Exercise 4.2: Instruction Following
Provide a VLM with an instruction (e.g., "Pick up the red cube") and a camera image. Generate a sequence of robot actions.

### Exercise 4.3: Task Decomposition
Build an agent that breaks down complex instructions:
- Input: "Set the table for dinner"
- Output: ["Place plate at position (x, y)", "Place fork to left of plate", "Place knife to right of plate"]

### Exercise 4.4: Closed-Loop Control
Implement a feedback loop:
1. Robot executes action
2. Camera captures result
3. VLM evaluates success
4. Refine next action if needed

### Exercise 4.5: Safety Constraints
Add a safety layer that validates actions before execution:
- Check joint limits
- Avoid collisions
- Verify grasp stability

## Code Example: VLA Agent (Pseudocode with Claude Vision)

```python
import anthropic
import base64
from robot_controller import RobotArm

def vla_agent(instruction: str, camera_image_path: str) -> list[str]:
    """
    Given a natural language instruction and a camera image,
    return a list of robot actions.
    """
    client = anthropic.Anthropic()
    
    # Read and encode the image
    with open(camera_image_path, "rb") as f:
        image_data = base64.standard_b64encode(f.read()).decode("utf-8")
    
    # Build the prompt
    prompt = f"""
    You are a robot control agent. You see the environment through a camera.
    
    Current scene: [IMAGE]
    
    Task: {instruction}
    
    Generate a sequence of robot actions to accomplish this task.
    Format each action as:
    - action_type: <move_arm | move_gripper | move_base>
    - target: <position or object name>
    - parameters: <any additional parameters>
    
    Be specific and precise. Prioritize safety.
    """
    
    # Call Claude Vision API
    message = client.messages.create(
        model="claude-3-5-sonnet-20241022",
        max_tokens=1024,
        messages=[
            {
                "role": "user",
                "content": [
                    {
                        "type": "image",
                        "source": {
                            "type": "base64",
                            "media_type": "image/jpeg",
                            "data": image_data,
                        },
                    },
                    {
                        "type": "text",
                        "text": prompt
                    }
                ],
            }
        ],
    )
    
    # Parse actions from response
    response_text = message.content[0].text
    actions = parse_actions(response_text)
    
    return actions

def parse_actions(text: str) -> list[str]:
    """Parse structured actions from VLM response."""
    actions = []
    lines = text.split('\n')
    for line in lines:
        if line.startswith('- action_type:'):
            actions.append(line)
    return actions

def execute_plan(robot: RobotArm, actions: list[str]):
    """Execute a sequence of actions on the robot."""
    for action in actions:
        print(f"Executing: {action}")
        # Parse and execute action
        # (implementation depends on robot API)
        robot.execute_action(action)

# Example usage
if __name__ == "__main__":
    robot = RobotArm()
    instruction = "Pick up the red cube and place it on the blue shelf"
    camera_image = "robot_camera.jpg"
    
    actions = vla_agent(instruction, camera_image)
    execute_plan(robot, actions)
```

---

**Next:** [Capstone Project](./capstone.md)
