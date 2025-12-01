"""
OpenAI Agent for Physical AI tasks.

Uses GPT-4o with function calling for robotics task execution and planning.
"""

import openai
import os
from typing import Optional, Any

# Define functions that GPT-4 can call
FUNCTIONS = [
    {
        "name": "get_robot_state",
        "description": "Get the current state of the robot (position, joint angles, gripper state)",
        "parameters": {
            "type": "object",
            "properties": {
                "joint_names": {
                    "type": "array",
                    "items": {"type": "string"},
                    "description": "Names of joints to query (empty = all)"
                }
            }
        }
    },
    {
        "name": "move_robot_to_pose",
        "description": "Move the robot to a specific pose in 3D space",
        "parameters": {
            "type": "object",
            "properties": {
                "position": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "Target position [x, y, z] in meters"
                },
                "orientation": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 4,
                    "maxItems": 4,
                    "description": "Target orientation as quaternion [qx, qy, qz, qw]"
                }
            },
            "required": ["position"]
        }
    },
    {
        "name": "control_gripper",
        "description": "Open or close the robot's gripper",
        "parameters": {
            "type": "object",
            "properties": {
                "action": {
                    "type": "string",
                    "enum": ["open", "close"],
                    "description": "Gripper action"
                },
                "force": {
                    "type": "number",
                    "description": "Grip force (0-100%)"
                }
            },
            "required": ["action"]
        }
    },
    {
        "name": "detect_objects",
        "description": "Use computer vision to detect objects in the scene",
        "parameters": {
            "type": "object",
            "properties": {
                "object_class": {
                    "type": "string",
                    "description": "Object type to detect (e.g., 'cube', 'cylinder', 'bottle')"
                }
            }
        }
    }
]


class OpenAIAgent:
    """An OpenAI-based agent for robotics tasks."""
    
    def __init__(self, api_key: Optional[str] = None):
        """Initialize the OpenAI agent."""
        self.api_key = api_key or os.getenv("OPENAI_GEMINI_KEY")
        if not self.api_key:
            raise ValueError("OPENAI_GEMINI_KEY environment variable not set")
        
        openai.api_key = self.api_key
        self.model = "gpt-4o-mini"
        self.system_prompt = """You are an intelligent robot control agent. You can use function calling to:
- Get the robot's current state
- Move the robot to target positions
- Control the gripper
- Detect objects in the environment

Always plan your actions carefully and verify that each step is safe."""
    
    def chat(self, user_message: str, conversation_history: Optional[list] = None) -> dict[str, Any]:
        """
        Chat with the agent (function calling mode).
        
        Args:
            user_message: The user's instruction
            conversation_history: Previous messages for context
        
        Returns:
            Agent response with any function calls
        """
        messages = conversation_history or []
        messages.append({"role": "user", "content": user_message})
        
        response = openai.chat.completions.create(
            model=self.model,
            messages=messages,
            functions=FUNCTIONS,
            function_call="auto",
            temperature=0.7
        )
        
        return {
            "response": response.choices[0].message,
            "has_function_call": response.choices[0].message.function_call is not None,
            "model": self.model
        }
    
    def execute_function(self, function_name: str, function_args: dict) -> dict[str, Any]:
        """
        Execute a function call from the agent.
        
        Args:
            function_name: Name of the function to execute
            function_args: Arguments for the function
        
        Returns:
            Function execution result
        """
        if function_name == "get_robot_state":
            return self._get_robot_state(function_args)
        elif function_name == "move_robot_to_pose":
            return self._move_robot_to_pose(function_args)
        elif function_name == "control_gripper":
            return self._control_gripper(function_args)
        elif function_name == "detect_objects":
            return self._detect_objects(function_args)
        else:
            return {"error": f"Unknown function: {function_name}"}
    
    def _get_robot_state(self, args: dict) -> dict:
        """Simulate getting robot state."""
        return {
            "position": [0.0, 0.0, 0.8],
            "orientation": [0.0, 0.0, 0.0, 1.0],
            "joint_angles": [0.0, -1.57, 1.57, -1.57, 0.0, 0.0],
            "gripper_state": "open",
            "timestamp": __import__("time").time()
        }
    
    def _move_robot_to_pose(self, args: dict) -> dict:
        """Simulate moving robot to pose."""
        position = args.get("position", [0.0, 0.0, 0.8])
        return {
            "status": "success",
            "target_position": position,
            "estimated_time": 2.5,
            "path_clear": True
        }
    
    def _control_gripper(self, args: dict) -> dict:
        """Simulate gripper control."""
        action = args.get("action", "open")
        force = args.get("force", 50)
        return {
            "status": "success",
            "action": action,
            "force_applied": force,
            "gripper_position": 1.0 if action == "open" else 0.0
        }
    
    def _detect_objects(self, args: dict) -> dict:
        """Simulate object detection."""
        object_class = args.get("object_class", "unknown")
        return {
            "detected_objects": [
                {
                    "class": object_class,
                    "position": [0.3, 0.2, 0.5],
                    "confidence": 0.92
                }
            ],
            "num_detections": 1
        }
