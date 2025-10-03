#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from drive_msgs.msg import DriveParam
from ackermann_msgs.msg import AckermannDriveStamped

class DriveParamToAckermann(Node):
    def __init__(self):
        super().__init__('drive_param_to_ackermann')
        
        # Subscribe to DriveParam from C++ keyboard controller
        self.subscription = self.create_subscription(
            DriveParam,
            'input/drive_param/keyboard',
            self.drive_param_callback,
            10
        )
        
        # Publish AckermannDriveStamped commands to Gazebo
        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            '/cmd_ackermann',
            10
        )
        
        self.get_logger().info('DriveParam to AckermannDriveStamped converter started')
        self.get_logger().info('Listening on: input/drive_param/keyboard (drive_msgs/DriveParam)')
        self.get_logger().info('Publishing to: /cmd_ackermann (ackermann_msgs/AckermannDriveStamped)')
        
    def drive_param_callback(self, drive_param_msg):
        # Create AckermannDriveStamped message
        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.header.stamp = self.get_clock().now().to_msg()
        ackermann_msg.header.frame_id = 'base_link'
        
        # Direct conversion from DriveParam to Ackermann
        # DriveParam.velocity -> AckermannDriveStamped.drive.speed
        ackermann_msg.drive.speed = drive_param_msg.velocity
        
        # DriveParam.angle -> AckermannDriveStamped.drive.steering_angle
        ackermann_msg.drive.steering_angle = drive_param_msg.angle
        
        # Set other fields to default values
        ackermann_msg.drive.acceleration = 0.0
        ackermann_msg.drive.jerk = 0.0
        ackermann_msg.drive.steering_angle_velocity = 0.0
        
        # Publish the converted message
        self.publisher.publish(ackermann_msg)
        
        # Log the conversion for debugging
        self.get_logger().debug(
            f'Converted: velocity={drive_param_msg.velocity:.3f}, angle={drive_param_msg.angle:.3f} '
            f'-> speed={ackermann_msg.drive.speed:.3f}, steering_angle={ackermann_msg.drive.steering_angle:.3f}'
        )

def main(args=None):
    rclpy.init(args=args)
    
    node = DriveParamToAckermann()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()