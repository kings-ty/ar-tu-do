#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from drive_msgs.msg import drive_param
import json
import os
import time
from datetime import datetime

class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')
        
        # 구독자 설정
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        
        self.drive_subscription = self.create_subscription(
            drive_param,
            '/input/drive_param/keyboard',
            self.drive_callback,
            10
        )
        
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        
        # 데이터 저장 변수
        self.current_pose = None
        self.current_drive = None
        self.current_laser = None
        self.waypoints = []
        self.training_data = []
        
        # 기록 제어
        self.recording = False
        self.last_record_time = 0
        self.record_interval = 0.1  # 0.1초마다 기록
        
        # 파일 경로 설정
        self.package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.data_dir = os.path.join(self.package_dir, 'data')
        os.makedirs(self.data_dir, exist_ok=True)
        
        # 타이머 설정
        self.timer = self.create_timer(0.1, self.record_data)
        
        # 서비스 또는 토픽으로 기록 제어 (간단히 시작/정지)
        self.get_logger().info('Waypoint Recorder 노드가 시작되었습니다.')
        self.get_logger().info('수동으로 차를 운전하면서 "/start_recording" 토픽에 메시지를 보내면 기록이 시작됩니다.')
        
        # 기록 제어 토픽
        self.control_subscription = self.create_subscription(
            PoseStamped,  # RViz에서 2D Nav Goal 사용
            '/goal_pose',
            self.control_callback,
            10
        )
        
    def pose_callback(self, msg):
        self.current_pose = msg
        
    def drive_callback(self, msg):
        self.current_drive = msg
        
    def laser_callback(self, msg):
        self.current_laser = msg
        
    def control_callback(self, msg):
        """RViz에서 목표 설정시 기록 시작/정지"""
        if not self.recording:
            self.start_recording()
        else:
            self.stop_recording()
    
    def start_recording(self):
        self.recording = True
        self.waypoints.clear()
        self.training_data.clear()
        self.get_logger().info('🔴 웨이포인트 기록 시작!')
        
    def stop_recording(self):
        if self.recording:
            self.recording = False
            self.save_data()
            self.get_logger().info(f'⏹️  기록 정지. 총 {len(self.waypoints)}개 웨이포인트 저장됨.')
    
    def record_data(self):
        if not self.recording:
            return
            
        current_time = time.time()
        if current_time - self.last_record_time < self.record_interval:
            return
            
        if (self.current_pose is None or 
            self.current_drive is None or 
            self.current_laser is None):
            return
            
        # 웨이포인트 데이터 기록
        waypoint = {
            'timestamp': current_time,
            'pose': {
                'x': self.current_pose.pose.pose.position.x,
                'y': self.current_pose.pose.pose.position.y,
                'z': self.current_pose.pose.pose.position.z,
                'qx': self.current_pose.pose.pose.orientation.x,
                'qy': self.current_pose.pose.pose.orientation.y,
                'qz': self.current_pose.pose.pose.orientation.z,
                'qw': self.current_pose.pose.pose.orientation.w
            }
        }
        
        # 머신러닝용 훈련 데이터 기록
        laser_data = list(self.current_laser.ranges)
        # inf 값을 큰 수로 변환
        laser_data = [x if x != float('inf') else 100.0 for x in laser_data]
        
        training_sample = {
            'timestamp': current_time,
            'laser_scan': laser_data,
            'drive_command': {
                'angle': self.current_drive.angle,
                'velocity': self.current_drive.velocity
            },
            'pose': waypoint['pose']
        }
        
        self.waypoints.append(waypoint)
        self.training_data.append(training_sample)
        self.last_record_time = current_time
        
        if len(self.waypoints) % 50 == 0:
            self.get_logger().info(f'기록된 웨이포인트: {len(self.waypoints)}개')
    
    def save_data(self):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # 웨이포인트 저장 (자율주행용)
        waypoint_file = os.path.join(self.data_dir, f'waypoints_{timestamp}.json')
        with open(waypoint_file, 'w') as f:
            json.dump(self.waypoints, f, indent=2)
        
        # 훈련 데이터 저장 (머신러닝용)
        training_file = os.path.join(self.data_dir, f'training_data_{timestamp}.json')
        with open(training_file, 'w') as f:
            json.dump(self.training_data, f, indent=2)
            
        self.get_logger().info(f'데이터 저장 완료:')
        self.get_logger().info(f'  웨이포인트: {waypoint_file}')
        self.get_logger().info(f'  훈련 데이터: {training_file}')

def main(args=None):
    rclpy.init(args=args)
    recorder = WaypointRecorder()
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.get_logger().info('키보드 인터럽트로 종료.')
        if recorder.recording:
            recorder.stop_recording()
    finally:
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()