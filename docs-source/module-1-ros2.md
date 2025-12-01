---
title: "Module 1: Robotic Nervous System (ROS 2)"
description: Learn the ROS 2 middleware, node architecture, and inter-process communication patterns.
tags: [ros2, middleware, nodes, topics, services]
---

# Module 1: Robotic Nervous System (ROS 2)

ROS 2 (Robot Operating System 2) is the de facto standard middleware for building robotic systems. Think of it as the "nervous system" of a robot—it enables different components (sensors, motors, planning algorithms) to communicate and coordinate seamlessly.

## Learning Objectives

- Understand ROS 2 architecture and the publish-subscribe model
- Create and run a basic ROS 2 node in Python
- Use topics, services, and action servers for inter-process communication
- Build a multi-node system with launch files
- Debug ROS 2 systems using command-line tools (ros2 topic, ros2 service)

## Key Concepts

### Nodes
A **node** is a process that performs a computation. Each node is independent and can run on different machines.

### Topics
A **topic** is a named bus over which nodes exchange messages. Topics use a publish-subscribe pattern:
- **Publishers** send data to a topic
- **Subscribers** receive data from a topic
- Communication is asynchronous

### Services
A **service** is a synchronous request-reply communication pattern:
- **Client** sends a request
- **Server** processes and replies
- Useful for one-off queries (e.g., "compute trajectory")

### Actions
An **action** is a goal-based communication pattern:
- **Client** sends a goal
- **Server** provides feedback and final result
- Useful for long-running tasks (e.g., "move to target")

## Reading

- [ROS 2 Official Documentation](https://docs.ros.org/en/iron/)
- [ROS 2 Python Tutorials](https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [Understanding ROS 2 Middleware](https://docs.ros.org/en/iron/Concepts/Intermediate/About-Middleware-Implementations.html)

## Hands-on Exercises

### Exercise 1.1: Hello Node
Create your first ROS 2 node that publishes "Hello, Robotics!" every second.

**Expected output:**
```
[INFO] Publishing: "Hello, Robotics!"
[INFO] Publishing: "Hello, Robotics!"
...
```

See `/examples/ros2/hello_node.py` for the solution.

### Exercise 1.2: Topic Listener
Create a subscriber node that listens to the publisher from Exercise 1.1 and prints messages.

### Exercise 1.3: Simple Service
Create a service server that computes the sum of two integers. Write a client that calls it.

### Exercise 1.4: Launch File
Create a ROS 2 launch file that starts both the publisher and subscriber from Exercises 1.1 and 1.2 simultaneously.

## Code Example: ROS 2 Publisher

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloPublisher(Node):
    def __init__(self):
        super().__init__('hello_publisher')
        self.publisher_ = self.create_publisher(String, 'hello_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, Robotics!'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = HelloPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

**Next:** [Module 2: Digital Twin & Simulation](./module-2-digital-twin.md)
