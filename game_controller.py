#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from drive_msgs.msg import DriveParam
import sys, select, termios, tty

class GameController(Node):
    def __init__(self):
        super().__init__('game_controller')
        
        self.drive_pub = self.create_publisher(
            DriveParam, 
            '/commands/drive_param', 
            10
        )
        
        self.current_angle = 0.0
        self.max_angle = 0.4
        self.speed = 1.0
        
        print("=== F1TENTH GAME CONTROLLER ===")
        print("Controls:")
        print("  w : Forward")
        print("  s : Backward") 
        print("  a : Turn Left")
        print("  d : Turn Right")
        print("  SPACE : Stop")
        print("  q : Quit")
        print("\nPress keys to control the car...")
        
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def publish_command(self, velocity, angle):
        drive_msg = DriveParam()
        drive_msg.velocity = float(velocity)
        drive_msg.angle = float(angle)
        self.drive_pub.publish(drive_msg)
        print(f"Vel: {velocity:.1f}, Angle: {angle:.2f}")
    
    def run(self):
        self.settings = termios.tcgetattr(sys.stdin)
        
        try:
            while True:
                key = self.get_key()
                
                if key == 'w':  # 전진
                    self.publish_command(self.speed, self.current_angle)
                    
                elif key == 's':  # 후진
                    self.publish_command(-self.speed, 0.0)  # 후진은 직진만
                    
                elif key == 'a':  # 좌회전
                    self.current_angle = self.max_angle
                    self.publish_command(0.0, self.current_angle)
                    
                elif key == 'd':  # 우회전
                    self.current_angle = -self.max_angle
                    self.publish_command(0.0, self.current_angle)
                    
                elif key == ' ':  # 정지 및 조향 리셋
                    self.current_angle = 0.0
                    self.publish_command(0.0, 0.0)
                    
                elif key == 'q':  # 종료
                    break
                    
                elif key == '':  # 키를 떼면 조향 중립
                    if self.current_angle != 0.0:
                        self.current_angle = 0.0
                        self.publish_command(0.0, 0.0)
                        
        except Exception as e:
            print(e)
        finally:
            # 종료 시 정지
            self.publish_command(0.0, 0.0)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main():
    rclpy.init()
    controller = GameController()
    
    try:
        controller.run()
    except KeyboardInterrupt:
        pass
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()