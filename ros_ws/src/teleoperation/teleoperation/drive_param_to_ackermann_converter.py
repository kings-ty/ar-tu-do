#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from drive_msgs.msg import DriveParam
from ackermann_msgs.msg import AckermannDriveStamped

class DriveParamConverter(Node):
    def __init__(self):
        super().__init__('drive_param_to_ackermann_converter')
        
        # Subscriber to the original keyboard controller topic
        self.subscription = self.create_subscription(
            DriveParam,
            'input/drive_param/keyboard',
            self.listener_callback,
            10)
        
        # Publisher for the topic the Gazebo Ackermann plugin expects
        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            '/racer/drive_param',
            10)
        
        self.get_logger().info('DriveParam to AckermannDriveStamped converter has started.')

    def listener_callback(self, drive_param_msg):
        # Create a new AckermannDriveStamped message
        ackermann_msg = AckermannDriveStamped()

        # Copy the header from the incoming message if it had one, or create a new one
        ackermann_msg.header.stamp = self.get_clock().now().to_msg()
        ackermann_msg.header.frame_id = 'base_link' # Or the appropriate frame

        # Translate the fields
        # Note: You might need to adjust signs or scale depending on robot orientation
        ackermann_msg.drive.steering_angle = drive_param_msg.angle
        ackermann_msg.drive.speed = drive_param_msg.velocity

        # Publish the converted message
        self.publisher.publish(ackermann_msg)

def main(args=None):
    rclpy.init(args=args)
    converter = DriveParamConverter()
    rclpy.spin(converter)
    converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
