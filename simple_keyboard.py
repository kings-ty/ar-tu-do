#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray  # Float64 대신 Float64MultiArray를 임포트
import sys
import tty
import termios
import select
import math

# --- 설정 값 ---
DRIVE_SPEED = 5.0             # 전진/후진 속도 (rad/s)
STEERING_ANGLE_MAX = 0.4    # 최대 조향각 (rad)
WHEEL_BASE = 0.32             # 차량 축간거리 (m)
WHEEL_TRACK = 0.211           # 차량 윤거 (m)

# --- 키보드 안내 메시지 ---
msg = """
-------------------------------------------
        w: 전진
a: 좌회전       d: 우회전
        s: 후진
        x: 조향 정렬

space: 정지
q: 종료
-------------------------------------------
"""

class SimpleKeyboard(Node):
    def __init__(self):
        super().__init__('simple_keyboard_controller')
        
        self.pub_lw = self.create_publisher(Float64MultiArray, '/left_wheel_controller/commands', 10)
        self.pub_rw = self.create_publisher(Float64MultiArray, '/right_wheel_controller/commands', 10)
        self.pub_ls = self.create_publisher(Float64MultiArray, '/left_steering_controller/commands', 10)
        self.pub_rs = self.create_publisher(Float64MultiArray, '/right_steering_controller/commands', 10)

        self.speed = 0.0
        self.steer_angle_center = 0.0
        print(msg)

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
        self.settings = termios.tcgetattr(sys.stdin)
        try:
            while rclpy.ok():
                key = self.get_key()

                if key == 'q' or key == 'Q':
                    break
                
                if key == 'w':
                    self.speed = DRIVE_SPEED
                elif key == 's':
                    self.speed = -DRIVE_SPEED
                elif key == ' ':
                    self.speed = 0.0
                    self.steer_angle_center = 0.0
                
                if key == 'a':
                    self.steer_angle_center = STEERING_ANGLE_MAX
                elif key == 'd':
                    self.steer_angle_center = -STEERING_ANGLE_MAX
                elif key == 'x':
                    self.steer_angle_center = 0.0

                if self.steer_angle_center != 0.0:
                    angle_outer = math.atan(WHEEL_BASE / (WHEEL_BASE / math.tan(abs(self.steer_angle_center)) + WHEEL_TRACK))
                    angle_inner = math.atan(WHEEL_BASE / (WHEEL_BASE / math.tan(abs(self.steer_angle_center)) - WHEEL_TRACK))
                    
                    if self.steer_angle_center > 0:
                        left_steer_cmd = angle_inner
                        right_steer_cmd = angle_outer
                    else:
                        left_steer_cmd = -angle_outer
                        right_steer_cmd = -angle_inner
                else:
                    left_steer_cmd = 0.0
                    right_steer_cmd = 0.0

                # 메시지 생성 (Float64MultiArray 타입으로)
                vel_cmd = Float64MultiArray()
                vel_cmd.data = [float(self.speed)]  # 데이터를 리스트/배열 형태로 전송
                
                left_steer_msg = Float64MultiArray()
                left_steer_msg.data = [float(left_steer_cmd)]
                
                right_steer_msg = Float64MultiArray()
                right_steer_msg.data = [float(right_steer_cmd)]

                # 메시지 퍼블리시
                self.pub_lw.publish(vel_cmd)
                self.pub_rw.publish(vel_cmd)
                self.pub_ls.publish(left_steer_msg)
                self.pub_rs.publish(right_steer_msg)
                
                rclpy.spin_once(self, timeout_sec=0.01)

        except Exception as e:
            print(e)
        finally:
            stop_cmd = Float64MultiArray()
            stop_cmd.data = [0.0]
            self.pub_lw.publish(stop_cmd)
            self.pub_rw.publish(stop_cmd)
            self.pub_ls.publish(stop_cmd)
            self.pub_rs.publish(stop_cmd)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main():
    rclpy.init()
    keyboard = SimpleKeyboard()
    keyboard.run()
    keyboard.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
