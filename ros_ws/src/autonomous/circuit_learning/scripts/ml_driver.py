#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from drive_msgs.msg import drive_param
import torch
import torch.nn as nn
import numpy as np
import os
import glob

class CircuitDrivingNet(nn.Module):
    """서킷 자율주행을 위한 신경망 (trainer와 동일)"""
    def __init__(self, input_size=360, hidden_size=128):
        super(CircuitDrivingNet, self).__init__()
        
        self.network = nn.Sequential(
            nn.Linear(input_size, hidden_size),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(hidden_size, hidden_size // 2),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(hidden_size // 2, hidden_size // 4),
            nn.ReLU(),
            nn.Dropout(0.1),
            nn.Linear(hidden_size // 4, 2),
            nn.Tanh()
        )
        
    def forward(self, x):
        return self.network(x)

class MLDriver(Node):
    def __init__(self):
        super().__init__('ml_driver')
        
        # 구독자 설정
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        
        # 퍼블리셔 설정
        self.drive_publisher = self.create_publisher(
            drive_param,
            '/input/drive_param/autonomous',
            10
        )
        
        # 디바이스 설정
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        # 모델 로드
        self.model = None
        self.load_latest_model()
        
        # 제어 파라미터
        self.max_velocity = 0.4
        self.min_velocity = 0.1
        self.driving = False
        
        self.get_logger().info('ML Driver 노드가 시작되었습니다.')
        if self.model is not None:
            self.get_logger().info('모델 로드 완료. 자율주행 준비됨.')
            self.driving = True
        else:
            self.get_logger().warn('모델이 없습니다. 먼저 훈련해주세요.')
    
    def load_latest_model(self):
        """가장 최근의 모델 로드"""
        package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        model_dir = os.path.join(package_dir, 'models')
        
        if not os.path.exists(model_dir):
            self.get_logger().warn('모델 디렉토리가 없습니다.')
            return
            
        model_files = glob.glob(os.path.join(model_dir, 'circuit_model_*.pth'))
        
        if not model_files:
            self.get_logger().warn('모델 파일이 없습니다. 먼저 훈련해주세요.')
            return
            
        # 가장 최근 파일 선택
        latest_model = sorted(model_files)[-1]
        
        try:
            # 모델 데이터 로드
            checkpoint = torch.load(latest_model, map_location=self.device)
            input_size = checkpoint['input_size']
            
            # 모델 생성 및 가중치 로드
            self.model = CircuitDrivingNet(input_size=input_size).to(self.device)
            self.model.load_state_dict(checkpoint['model_state_dict'])
            self.model.eval()
            
            self.get_logger().info(f'모델 로드 완료: {os.path.basename(latest_model)}')
            
        except Exception as e:
            self.get_logger().error(f'모델 로드 실패: {e}')
            self.model = None
    
    def preprocess_laser_data(self, laser_ranges):
        """LiDAR 데이터 전처리"""
        # numpy 배열로 변환
        laser_array = np.array(laser_ranges)
        
        # inf 값을 큰 수로 변환
        laser_array = np.where(np.isinf(laser_array), 100.0, laser_array)
        
        # 거리 제한 및 정규화
        laser_array = np.clip(laser_array, 0, 10)
        laser_array = laser_array / 10.0
        
        return laser_array
    
    def laser_callback(self, msg):
        """LiDAR 데이터 수신 및 제어 명령 생성"""
        if not self.driving or self.model is None:
            return
            
        # 데이터 전처리
        laser_data = self.preprocess_laser_data(msg.ranges)
        
        # 텐서로 변환
        input_tensor = torch.FloatTensor(laser_data).unsqueeze(0).to(self.device)
        
        # 모델 추론
        with torch.no_grad():
            output = self.model(input_tensor)
            
        # 출력 해석
        angle = output[0][0].item()  # -1 ~ 1
        velocity_norm = output[0][1].item()  # -1 ~ 1
        
        # 속도 변환 (정규화된 값을 실제 속도로)
        velocity = ((velocity_norm + 1) / 2) * (self.max_velocity - self.min_velocity) + self.min_velocity
        
        # 드라이브 명령 생성
        drive_msg = drive_param()
        drive_msg.angle = float(angle)
        drive_msg.velocity = float(velocity)
        
        # 명령 전송
        self.drive_publisher.publish(drive_msg)
        
        # 주기적 로그
        if hasattr(self, 'log_counter'):
            self.log_counter += 1
        else:
            self.log_counter = 0
            
        if self.log_counter % 50 == 0:  # 5초마다 로그
            self.get_logger().info(f'ML 제어: 각도={angle:.3f}, 속도={velocity:.3f}')

def main(args=None):
    rclpy.init(args=args)
    driver = MLDriver()
    
    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        driver.get_logger().info('키보드 인터럽트로 종료.')
    finally:
        driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()