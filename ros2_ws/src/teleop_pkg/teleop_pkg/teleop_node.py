#!/usr/bin/env python3
"""
Teleop Node - Keyboard control for the robot
Controls robot movement using W/A/S/D keys

W = forward
S = backward
A = turn left
D = turn right
Space = stop
Q = quit

Author: Benjamin Matapo
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        
        # Create publisher for velocity commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Robot speed settings
        self.linear_speed = 0.5   # m/s - forward/backward speed
        self.angular_speed = 1.0  # rad/s - turning speed
        self.current_speed = 0.0
        
        # Print instructions
        self.get_logger().info('Teleop Node Started!')
        self.print_instructions()
    
    def print_instructions(self):
        """Print keyboard control instructions"""
        instructions = """
        ================================
        Keyboard Control Instructions:
        ================================
        W - Move Forward
        S - Move Backward
        A - Turn Left
        D - Turn Right
        Space - Stop
        Q - Quit
        ================================
        """
        print(instructions)
    
    def get_key(self):
        """
        Capture a single keypress from keyboard
        This is tricky because normally terminal waits for Enter
        """
        # Save current terminal settings
        settings = termios.tcgetattr(sys.stdin.fileno())
        
        try:
            # Change terminal to raw mode (reads keys immediately)
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
        
        return key
    
    def run(self):
        """Main control loop"""
        self.get_logger().info('Waiting for keyboard input...')
        
        try:
            while rclpy.ok():
                # Get keyboard input
                key = self.get_key()
                
                # Create velocity message
                msg = Twist()
                
                # Process the key and set velocities
                if key == 'w' or key == 'W':
                    # Move forward
                    msg.linear.x = self.linear_speed
                    msg.angular.z = 0.0
                    self.get_logger().info('Moving FORWARD')
                
                elif key == 's' or key == 'S':
                    # Move backward
                    msg.linear.x = -self.linear_speed
                    msg.angular.z = 0.0
                    self.get_logger().info('Moving BACKWARD')
                
                elif key == 'a' or key == 'A':
                    # Turn left
                    msg.linear.x = 0.0
                    msg.angular.z = self.angular_speed
                    self.get_logger().info('Turning LEFT')
                
                elif key == 'd' or key == 'D':
                    # Turn right
                    msg.linear.x = 0.0
                    msg.angular.z = -self.angular_speed
                    self.get_logger().info('Turning RIGHT')
                
                elif key == ' ':
                    # Stop
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                    self.get_logger().info('STOPPED')
                
                elif key == 'q' or key == 'Q':
                    # Quit
                    self.get_logger().info('Quitting...')
                    # Send stop command before quitting
                    stop_msg = Twist()
                    self.publisher.publish(stop_msg)
                    break
                
                else:
                    # Unknown key
                    self.get_logger().warn(f'Unknown key: {key}')
                    continue
                
                # Publish the velocity command
                self.publisher.publish(msg)
        
        except KeyboardInterrupt:
            # Handle Ctrl+C gracefully
            self.get_logger().info('Interrupted by user')
        
        finally:
            # Always send stop command when exiting
            stop_msg = Twist()
            self.publisher.publish(stop_msg)
            self.get_logger().info('Teleop node stopped')


def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create the teleop node
    node = TeleopNode()
    
    # Run the node
    node.run()
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
