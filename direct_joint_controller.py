#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from drive_msgs.msg import DriveParam
from std_msgs.msg import Float64MultiArray
import sys
import tty
import termios
import select

class DirectJointController(Node):
    def __init__(self):
        super().__init__('direct_joint_controller')
        
        # Subscribe to DriveParam
        self.subscription = self.create_subscription(
            DriveParam,
            'input/drive_param/keyboard',
            self.drive_param_callback,
            10
        )
        
        # Publishers for direct joint control
        self.wheel_pub = self.create_publisher(
            Float64MultiArray,
            '/effort_controller/commands',
            10
        )
        
        self.steering_pub = self.create_publisher(
            Float64MultiArray, 
            '/steering_controller/commands',
            10
        )
        
        self.get_logger().info('Direct Joint Controller started')
        self.get_logger().info('Directly controlling wheel and steering joints')
        
    def drive_param_callback(self, drive_param_msg):
        # Control rear wheels for driving
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [
            drive_param_msg.velocity * 10.0,  # left rear wheel
            drive_param_msg.velocity * 10.0   # right rear wheel
        ]
        
        # Control front steering
        steering_msg = Float64MultiArray()
        steering_msg.data = [
            drive_param_msg.angle,  # left steering
            drive_param_msg.angle   # right steering
        ]
        
        self.wheel_pub.publish(wheel_msg)
        self.steering_pub.publish(steering_msg)
        
        if abs(drive_param_msg.velocity) > 0.01 or abs(drive_param_msg.angle) > 0.01:
            self.get_logger().info(
                f'Direct control: vel={drive_param_msg.velocity:.2f}, steer={drive_param_msg.angle:.2f}'
            )

class DirectAckermannTeleop(Node):
    def __init__(self):
        super().__init__('direct_ackermann_teleop')
        
        self.publisher = self.create_publisher(DriveParam, 'input/drive_param/keyboard', 10)
        
        self.max_velocity = 0.3
        self.max_angle = 0.5
        
        print("=== Direct Ackermann Control ===")
        print("Direct joint control for F1 car")
        print("W - Forward, S - Backward")
        print("A - Left steer, D - Right steer") 
        print("X - Stop, Q - Quit")
        print("-" * 40)
        
        self.settings = termios.tcgetattr(sys.stdin)
        
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
        
    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                
                msg = DriveParam()
                
                if key == 'q' or key == 'Q':
                    break
                elif key == 'w' or key == 'W':
                    msg.velocity = self.max_velocity
                    msg.angle = 0.0
                    print(f"Forward: {msg.velocity:.2f}")
                elif key == 's' or key == 'S':
                    msg.velocity = -self.max_velocity * 0.5
                    msg.angle = 0.0
                    print(f"Backward: {msg.velocity:.2f}")
                elif key == 'a' or key == 'A':
                    msg.velocity = 0.0
                    msg.angle = self.max_angle
                    print(f"Left: {msg.angle:.2f}")
                elif key == 'd' or key == 'D':
                    msg.velocity = 0.0
                    msg.angle = -self.max_angle
                    print(f"Right: {msg.angle:.2f}")
                elif key == 'x' or key == 'X':
                    msg.velocity = 0.0
                    msg.angle = 0.0
                    print("Stop")
                else:
                    msg.velocity = 0.0
                    msg.angle = 0.0
                
                self.publisher.publish(msg)
                rclpy.spin_once(self, timeout_sec=0.01)
                
        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main():
    rclpy.init()
    
    # Start both controller and teleop
    controller = DirectJointController()
    teleop = DirectAckermannTeleop() 
    
    try:
        # Run teleop in main thread
        teleop.run()
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        teleop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()