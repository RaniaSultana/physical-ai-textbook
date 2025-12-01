#!/usr/bin/env python3
"""
Simple ROS 2 Hello Node example.

This is a minimal ROS 2 publisher node that sends "Hello, Robotics!" 
to a topic every second.

Corresponding to Exercise 1.1 in Module 1: Robotic Nervous System.

Usage:
    ros2 run examples hello_node.py

Or if not in a ROS 2 package:
    python3 hello_node.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HelloPublisher(Node):
    """A simple publisher node that sends hello messages."""
    
    def __init__(self):
        super().__init__('hello_publisher')
        
        # Create a publisher on topic 'hello_topic' with a queue size of 10
        self.publisher_ = self.create_publisher(String, 'hello_topic', 10)
        
        # Create a timer to publish messages every 1.0 seconds
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info('Hello Publisher initialized. Publishing to /hello_topic')
    
    def timer_callback(self):
        """Callback function called every time the timer fires."""
        msg = String()
        msg.data = 'Hello, Robotics!'
        
        # Publish the message
        self.publisher_.publish(msg)
        
        # Log the action
        self.get_logger().info(f'Publishing: "{msg.data}"')


def main(args=None):
    """Main entry point for the node."""
    # Initialize ROS 2
    rclpy.init(args=args)
    
    # Create the publisher node
    node = HelloPublisher()
    
    # Keep the node running (spin) until interrupted (Ctrl+C)
    rclpy.spin(node)
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
