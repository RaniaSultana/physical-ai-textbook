"""
Claude Agent for Physical AI tasks.

Uses Claude 3.5 Sonnet with tool use for robotics task planning and execution.
"""

import anthropic
import os
from typing import Optional, Any

# Define tools that Claude can use
TOOLS = [
    {
        "name": "plan_robot_motion",
        "description": "Plan a sequence of robot motions to achieve a task",
        "input_schema": {
            "type": "object",
            "properties": {
                "task": {
                    "type": "string",
                    "description": "The task to plan (e.g., 'pick up the red cube')"
                },
                "constraints": {
                    "type": "array",
                    "items": {"type": "string"},
                    "description": "Constraints (e.g., 'avoid collision', 'use gripper')"
                }
            },
            "required": ["task"]
        }
    },
    {
        "name": "execute_ros_command",
        "description": "Execute a ROS 2 command on the robot",
        "input_schema": {
            "type": "object",
            "properties": {
                "command": {
                    "type": "string",
                    "description": "ROS 2 command (e.g., 'ros2 service call /move_arm')"
                },
                "parameters": {
                    "type": "object",
                    "description": "Command parameters"
                }
            },
            "required": ["command"]
        }
    },
    {
        "name": "query_sensor_data",
        "description": "Query sensor data from the robot",
        "input_schema": {
            "type": "object",
            "properties": {
                "sensor_type": {
                    "type": "string",
                    "enum": ["camera", "lidar", "imu", "joint_states"],
                    "description": "Type of sensor to query"
                }
            },
            "required": ["sensor_type"]
        }
    }
]


class ClaudeAgent:
    """A Claude-based agent for robotics tasks."""
    
    def __init__(self, api_key: Optional[str] = None):
        """Initialize the Claude agent."""
        self.api_key = api_key or os.getenv("CLAUDE_CODE_API_KEY")
        if not self.api_key:
            raise ValueError("CLAUDE_CODE_API_KEY environment variable not set")
        
        self.client = anthropic.Anthropic(api_key=self.api_key)
        self.model = "claude-3-5-sonnet-20241022"
        self.system_prompt = """You are a robotics assistant that helps plan and execute tasks on a humanoid robot.
You can use tools to plan motions, execute ROS commands, and query sensor data.
Always verify actions are safe before executing them."""
    
    def plan_task(self, task: str, context: Optional[str] = None) -> dict[str, Any]:
        """
        Use Claude to plan a task.
        
        Args:
            task: The task to plan (e.g., "pick up the red cube")
            context: Optional context about the environment
        
        Returns:
            A dict with 'plan' (step-by-step actions) and 'reasoning'
        """
        user_message = f"Plan the following task: {task}"
        if context:
            user_message += f"\n\nContext: {context}"
        
        response = self.client.messages.create(
            model=self.model,
            max_tokens=1024,
            system=self.system_prompt,
            tools=TOOLS,
            messages=[
                {"role": "user", "content": user_message}
            ]
        )
        
        # Extract text and tool calls
        plan = []
        reasoning = ""
        
        for block in response.content:
            if hasattr(block, "text"):
                reasoning = block.text
            elif block.type == "tool_use":
                plan.append({
                    "tool": block.name,
                    "input": block.input
                })
        
        return {
            "plan": plan,
            "reasoning": reasoning,
            "model": self.model
        }
    
    def execute_plan(self, plan: list[dict]) -> dict[str, Any]:
        """
        Execute a task plan.
        
        Args:
            plan: List of tool calls from plan_task()
        
        Returns:
            Execution results
        """
        results = []
        for step in plan:
            tool_name = step["tool"]
            tool_input = step["input"]
            
            # Simulate tool execution
            if tool_name == "plan_robot_motion":
                result = self._simulate_motion_planning(tool_input)
            elif tool_name == "execute_ros_command":
                result = self._simulate_ros_command(tool_input)
            elif tool_name == "query_sensor_data":
                result = self._simulate_sensor_query(tool_input)
            else:
                result = {"error": f"Unknown tool: {tool_name}"}
            
            results.append({"tool": tool_name, "result": result})
        
        return {
            "executed_steps": len(results),
            "results": results,
            "status": "completed"
        }
    
    def _simulate_motion_planning(self, params: dict) -> dict:
        """Simulate motion planning (placeholder)."""
        return {
            "task": params.get("task"),
            "waypoints": [
                {"position": [0.0, 0.0, 0.5], "name": "start"},
                {"position": [0.5, 0.2, 0.3], "name": "target"}
            ],
            "estimated_time": 3.5
        }
    
    def _simulate_ros_command(self, params: dict) -> dict:
        """Simulate ROS command execution (placeholder)."""
        return {
            "command": params.get("command"),
            "status": "success",
            "output": "Command executed successfully"
        }
    
    def _simulate_sensor_query(self, params: dict) -> dict:
        """Simulate sensor data query (placeholder)."""
        sensor_type = params.get("sensor_type")
        
        if sensor_type == "camera":
            return {"type": "camera", "resolution": [1920, 1080], "fps": 30}
        elif sensor_type == "lidar":
            return {"type": "lidar", "range": 30.0, "points": 72000}
        elif sensor_type == "imu":
            return {"type": "imu", "accel": [0.0, 0.0, -9.81], "angular_vel": [0.0, 0.0, 0.0]}
        elif sensor_type == "joint_states":
            return {"type": "joint_states", "num_joints": 12, "positions": [0.0] * 12}
        else:
            return {"error": f"Unknown sensor: {sensor_type}"}
