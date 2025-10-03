#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from drive_msgs.msg import DriveParam
from geometry_msgs.msg import Twist

class QuickFixConverter(Node):
    def __init__(self):
        super().__init__('quick_fix_converter')
        
        # Subscribe to DriveParam from C++ keyboard controller
        self.subscription = self.create_subscription(
            DriveParam,
            'input/drive_param/keyboard',
            self.drive_param_callback,
            10
        )
        
        # Publish to existing differential drive plugin
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_ackermann_converted',
            10
        )
        
        self.get_logger().info('Quick Fix: DriveParam to Twist converter started')
        self.get_logger().info('Publishing to: /cmd_ackermann_converted (geometry_msgs/Twist)')
        
    def drive_param_callback(self, drive_param_msg):
        twist_msg = Twist()
        
        # Direct conversion for quick test
        twist_msg.linear.x = drive_param_msg.velocity
        twist_msg.angular.z = drive_param_msg.angle * 2.0  # Scale steering for differential drive
        
        self.publisher.publish(twist_msg)
        
        if abs(drive_param_msg.velocity) > 0.01 or abs(drive_param_msg.angle) > 0.01:
            self.get_logger().info(
                f'Converting: vel={drive_param_msg.velocity:.2f}, angle={drive_param_msg.angle:.2f} '
                f'-> linear.x={twist_msg.linear.x:.2f}, angular.z={twist_msg.angular.z:.2f}'
            )

def main():
    rclpy.init()
    converter = QuickFixConverter()
    try:
        rclpy.spin(converter)
    except KeyboardInterrupt:
        pass
    finally:
        converter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()