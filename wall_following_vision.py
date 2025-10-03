#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionWallFollower(Node):
    def __init__(self):
        super().__init__('vision_wall_follower')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.image_callback,
            10
        )
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Control parameters
        self.linear_speed = 1.0
        self.max_angular_speed = 1.0
        
        # PID parameters for steering
        self.kp = 0.01
        self.ki = 0.0
        self.kd = 0.005
        self.prev_error = 0.0
        self.integral = 0.0
        
        self.get_logger().info('Vision Wall Follower started')
    
    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Process image to find track boundaries
            steering_angle = self.process_image(cv_image)
            
            # Publish control command
            self.publish_control_command(steering_angle)
            
        except Exception as e:
            self.get_logger().error(f'Image processing error: {str(e)}')
    
    def process_image(self, image):
        """Process camera image to determine steering angle"""
        
        # Image dimensions
        height, width = image.shape[:2]
        
        # Focus on bottom half of image (road area)
        roi_top = height // 2
        roi = image[roi_top:, :]
        
        # Convert to HSV for better color filtering
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Define range for track surface (adjust these values based on your track)
        # Assuming track is grayish and walls are different colors
        track_lower = np.array([0, 0, 50])    # Lower gray
        track_upper = np.array([180, 50, 200]) # Upper gray
        
        # Create mask for track surface
        track_mask = cv2.inRange(hsv, track_lower, track_upper)
        
        # Apply morphological operations to clean up mask
        kernel = np.ones((5,5), np.uint8)
        track_mask = cv2.morphologyEx(track_mask, cv2.MORPH_CLOSE, kernel)
        track_mask = cv2.morphologyEx(track_mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(track_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return 0.0  # No track found, go straight
        
        # Find largest contour (should be the track)
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Calculate moments to find centroid
        M = cv2.moments(largest_contour)
        
        if M["m00"] == 0:
            return 0.0
        
        # Calculate centroid x-coordinate
        track_center_x = int(M["m10"] / M["m00"])
        
        # Image center
        image_center_x = width // 2
        
        # Calculate error (deviation from center)
        error = track_center_x - image_center_x
        
        # PID control for steering
        self.integral += error
        derivative = error - self.prev_error
        
        steering_output = (self.kp * error + 
                          self.ki * self.integral + 
                          self.kd * derivative)
        
        self.prev_error = error
        
        # Normalize steering angle
        steering_angle = np.clip(steering_output / width * 2.0, -self.max_angular_speed, self.max_angular_speed)
        
        self.get_logger().info(f'Track center: {track_center_x}, Error: {error}, Steering: {steering_angle:.3f}')
        
        return steering_angle
    
    def publish_control_command(self, steering_angle):
        """Publish Twist command"""
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = -steering_angle  # Negative for correct turning direction
        
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    
    wall_follower = VisionWallFollower()
    
    try:
        rclpy.spin(wall_follower)
    except KeyboardInterrupt:
        pass
    
    wall_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()