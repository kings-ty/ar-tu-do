#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray
import numpy as np
import math
import pickle
import time
import os
import argparse
import h5py
import json
from collections import deque

class MoveToCenterTeacher:
    """move_to_center 로직을 내장한 Teacher 클래스"""
    def __init__(self, target_speed=40.0):
        # 제어 파라미터 (move_to_center.py와 동일)
        self.DRIVE_SPEED = target_speed
        self.MAX_STEERING_ANGLE = 0.3
        self.WHEEL_BASE = 0.32
        self.WHEEL_TRACK = 0.211
        
        # 트랙 폭 (측정된 값)
        self.TRACK_WIDTH = 1.368
        self.TARGET_CENTER_DISTANCE = self.TRACK_WIDTH / 2.0
        
    def get_distance_at_angle(self, scan, angle_degrees):
        """특정 각도에서의 거리 측정"""
        angle_rad = math.radians(angle_degrees)
        
        sample_count = len(scan.ranges)
        
        if scan.angle_max != scan.angle_min:
            index = int((angle_rad - scan.angle_min) / (scan.angle_max - scan.angle_min) * sample_count)
        else:
            return None
        
        if 0 <= index < sample_count:
            distance = scan.ranges[index]
            if scan.range_min <= distance <= scan.range_max:
                return distance
        
        return None
    
    def calculate_ackermann_steering(self, steer_angle_center):
        """Ackermann 조향 기하학 계산"""
        if steer_angle_center != 0.0:
            angle_outer = math.atan(self.WHEEL_BASE / (self.WHEEL_BASE / math.tan(abs(steer_angle_center)) + self.WHEEL_TRACK))
            angle_inner = math.atan(self.WHEEL_BASE / (self.WHEEL_BASE / math.tan(abs(steer_angle_center)) - self.WHEEL_TRACK))
            
            if steer_angle_center > 0:
                left_steer_cmd = angle_inner
                right_steer_cmd = angle_outer
            else:
                left_steer_cmd = -angle_outer
                right_steer_cmd = -angle_inner
        else:
            left_steer_cmd = 0.0
            right_steer_cmd = 0.0
        
        return left_steer_cmd, right_steer_cmd
    
    def make_decision(self, lidar_msg):
        """move_to_center 로직으로 조향/속도 결정"""
        # 좌우 90도 거리 측정
        right_distance = self.get_distance_at_angle(lidar_msg, -90)
        left_distance = self.get_distance_at_angle(lidar_msg, 90)
        
        if right_distance is None or left_distance is None:
            return None, None, None  # 데이터 없음
        
        # 위치 오차 계산
        position_error = right_distance - left_distance
        
        # 조향각 계산
        if abs(position_error) < 0.05:
            steering_angle = 0.0
            speed = self.DRIVE_SPEED
        elif position_error > 0:
            steering_angle = -min(abs(position_error) * 0.5, self.MAX_STEERING_ANGLE)
            speed = self.DRIVE_SPEED
        else:
            steering_angle = min(abs(position_error) * 0.5, self.MAX_STEERING_ANGLE)
            speed = self.DRIVE_SPEED
        
        # Ackermann 조향 계산
        left_steer, right_steer = self.calculate_ackermann_steering(steering_angle)
        
        return steering_angle, speed, {
            'left_distance': left_distance,
            'right_distance': right_distance,
            'position_error': position_error,
            'left_steer': left_steer,
            'right_steer': right_steer
        }

class SupervisedDataCollector(Node):
    def __init__(self, target_speed=40.0, collection_duration=300, output_dir="./collected_data", 
                 use_compression=True, data_format='hdf5'):
        super().__init__('supervised_data_collector')
        
        # Teacher 초기화
        self.teacher = MoveToCenterTeacher(target_speed)
        
        # 수집 파라미터
        self.target_speed = target_speed
        self.collection_duration = collection_duration
        self.output_dir = output_dir
        self.start_time = time.time()
        self.use_compression = use_compression
        self.data_format = data_format  # 'hdf5', 'npz', 'pickle'
        
        # 데이터 저장 (메모리 효율성을 위해 배열로 미리 할당)
        self.max_samples = int(collection_duration * 10)  # 10Hz 가정
        self.lidar_points = None  # 첫 번째 스캔에서 결정
        self.lidar_data = None
        self.steering_data = np.zeros(self.max_samples, dtype=np.float32)
        self.speed_data = np.zeros(self.max_samples, dtype=np.float32)
        self.metadata_list = []
        self.timestamps = np.zeros(self.max_samples, dtype=np.float64)
        
        self.sample_count = 0
        self.last_save_time = time.time()
        self.save_interval = 30  # 30초마다 중간 저장
        
        # 데이터 품질 관리
        self.steering_history = deque(maxlen=10)
        self.quality_threshold = {
            'max_steering_change': 0.2,  # 급격한 조향 변화 제한
            'min_wall_distance': 0.3,    # 너무 가까운 벽 제외
            'max_wall_distance': 3.0     # 너무 먼 벽 제외
        }
        
        # ROS 구독자
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        # 제어 명령 발행자 (실제 차량 제어용)
        self.pub_lw = self.create_publisher(Float64MultiArray, '/left_wheel_controller/commands', 10)
        self.pub_rw = self.create_publisher(Float64MultiArray, '/right_wheel_controller/commands', 10)
        self.pub_ls = self.create_publisher(Float64MultiArray, '/left_steering_controller/commands', 10)
        self.pub_rs = self.create_publisher(Float64MultiArray, '/right_steering_controller/commands', 10)
        
        # 출력 디렉토리 생성
        os.makedirs(output_dir, exist_ok=True)
        
        self.get_logger().info(f'=== 데이터 수집 시작 ===')
        self.get_logger().info(f'목표 속도: {target_speed}')
        self.get_logger().info(f'수집 시간: {collection_duration}초')
        self.get_logger().info(f'출력 디렉토리: {output_dir}')
    
    def preprocess_lidar_data(self, lidar_msg):
        """LiDAR 데이터 전처리 (ML 학습용)"""
        ranges = np.array(lidar_msg.ranges)
        
        # inf 값을 큰 수로 변환
        ranges = np.where(np.isinf(ranges), 10.0, ranges)
        
        # 거리 제한 및 정규화
        ranges = np.clip(ranges, 0, 10.0)
        ranges = ranges / 10.0  # 0-1 범위로 정규화
        
        return ranges.tolist()
    
    def is_good_sample(self, steering_angle, metadata):
        """데이터 품질 검증"""
        # 1. 급격한 조향 변화 체크
        if len(self.steering_history) > 0:
            last_steering = self.steering_history[-1]
            if abs(steering_angle - last_steering) > self.quality_threshold['max_steering_change']:
                return False
        
        # 2. 벽 거리 체크
        left_dist = metadata['left_distance']
        right_dist = metadata['right_distance']
        
        if (left_dist < self.quality_threshold['min_wall_distance'] or 
            right_dist < self.quality_threshold['min_wall_distance'] or
            left_dist > self.quality_threshold['max_wall_distance'] or
            right_dist > self.quality_threshold['max_wall_distance']):
            return False
        
        # 3. 극단적인 조향각 제외
        if abs(steering_angle) > 0.25:
            return False
        
        return True
    
    def publish_commands(self, speed, left_steer, right_steer):
        """실제 차량 제어 명령 발행"""
        # 속도 명령
        vel_cmd = Float64MultiArray()
        vel_cmd.data = [float(speed)]
        
        # 조향 명령
        left_steer_msg = Float64MultiArray()
        left_steer_msg.data = [float(left_steer)]
        
        right_steer_msg = Float64MultiArray()
        right_steer_msg.data = [float(right_steer)]
        
        # 발행
        self.pub_lw.publish(vel_cmd)
        self.pub_rw.publish(vel_cmd)
        self.pub_ls.publish(left_steer_msg)
        self.pub_rs.publish(right_steer_msg)
    
    def scan_callback(self, lidar_msg):
        """LiDAR 콜백 - 데이터 수집 및 차량 제어"""
        current_time = time.time()
        
        # 수집 시간 체크
        if current_time - self.start_time > self.collection_duration:
            self.finish_collection()
            return
        
        # Teacher의 결정
        steering_angle, speed, metadata = self.teacher.make_decision(lidar_msg)
        
        if steering_angle is None:
            return  # 유효하지 않은 데이터
        
        # 실제 차량 제어
        self.publish_commands(speed, metadata['left_steer'], metadata['right_steer'])
        
        # 데이터 품질 검증
        if self.is_good_sample(steering_angle, metadata) and self.sample_count < self.max_samples:
            # ML 학습용 데이터 생성
            processed_lidar = self.preprocess_lidar_data(lidar_msg)
            
            # 첫 번째 샘플에서 LiDAR 크기 결정
            if self.lidar_data is None:
                self.lidar_points = len(processed_lidar)
                self.lidar_data = np.zeros((self.max_samples, self.lidar_points), dtype=np.float32)
                self.get_logger().info(f'LiDAR 포인트 수: {self.lidar_points}')
            
            # 배열에 직접 저장 (메모리 효율적)
            self.timestamps[self.sample_count] = current_time
            self.lidar_data[self.sample_count] = processed_lidar
            self.steering_data[self.sample_count] = steering_angle
            self.speed_data[self.sample_count] = speed
            self.metadata_list.append(metadata)
            
            self.sample_count += 1
            self.steering_history.append(steering_angle)
            
            # 진행 상황 출력
            if self.sample_count % 100 == 0:
                elapsed = current_time - self.start_time
                remaining = self.collection_duration - elapsed
                self.get_logger().info(
                    f'수집된 샘플: {self.sample_count}, '
                    f'경과시간: {elapsed:.1f}s, '
                    f'남은시간: {remaining:.1f}s'
                )
        
        # 중간 저장
        if current_time - self.last_save_time > self.save_interval:
            self.save_intermediate()
            self.last_save_time = current_time
    
    def save_data_hdf5(self, filepath, is_final=False):
        """HDF5 포맷으로 데이터 저장 (고압축)"""
        if self.sample_count == 0:
            return
        
        # 실제 사용된 데이터만 슬라이싱
        actual_samples = self.sample_count
        
        compression = 'gzip' if self.use_compression else None
        compression_opts = 9 if self.use_compression else None
        
        with h5py.File(filepath, 'w') as f:
            # 메인 데이터 (고압축)
            f.create_dataset('timestamps', data=self.timestamps[:actual_samples], 
                           compression=compression, compression_opts=compression_opts)
            f.create_dataset('lidar_data', data=self.lidar_data[:actual_samples], 
                           compression=compression, compression_opts=compression_opts)
            f.create_dataset('steering_data', data=self.steering_data[:actual_samples], 
                           compression=compression, compression_opts=compression_opts)
            f.create_dataset('speed_data', data=self.speed_data[:actual_samples], 
                           compression=compression, compression_opts=compression_opts)
            
            # 메타데이터 (JSON 문자열로 저장 - 압축 옵션 제외)
            metadata_json = json.dumps(self.metadata_list[:actual_samples])
            f.create_dataset('metadata', data=metadata_json)
            
            # 설정 정보
            f.attrs['target_speed'] = self.target_speed
            f.attrs['data_format'] = 'hdf5_compressed' if self.use_compression else 'hdf5'
            f.attrs['lidar_points'] = self.lidar_points if self.lidar_points else 0
            f.attrs['sample_count'] = actual_samples
    
    def save_data_npz(self, filepath):
        """NumPy 압축 포맷으로 저장"""
        actual_samples = self.sample_count
        
        if self.use_compression:
            np.savez_compressed(filepath,
                              timestamps=self.timestamps[:actual_samples],
                              lidar_data=self.lidar_data[:actual_samples],
                              steering_data=self.steering_data[:actual_samples],
                              speed_data=self.speed_data[:actual_samples],
                              metadata=self.metadata_list[:actual_samples],
                              target_speed=self.target_speed)
        else:
            np.savez(filepath,
                    timestamps=self.timestamps[:actual_samples],
                    lidar_data=self.lidar_data[:actual_samples],
                    steering_data=self.steering_data[:actual_samples],
                    speed_data=self.speed_data[:actual_samples],
                    metadata=self.metadata_list[:actual_samples],
                    target_speed=self.target_speed)
    
    def save_intermediate(self):
        """중간 저장 (데이터 손실 방지)"""
        if self.sample_count == 0 or self.lidar_data is None:
            return
        
        timestamp = int(time.time())
        
        if self.data_format == 'hdf5':
            filename = f"collected_data_speed_{self.target_speed}_temp_{timestamp}.h5"
            filepath = os.path.join(self.output_dir, filename)
            self.save_data_hdf5(filepath)
        elif self.data_format == 'npz':
            filename = f"collected_data_speed_{self.target_speed}_temp_{timestamp}.npz"
            filepath = os.path.join(self.output_dir, filename)
            self.save_data_npz(filepath)
        else:  # pickle fallback
            filename = f"collected_data_speed_{self.target_speed}_temp_{timestamp}.pkl"
            filepath = os.path.join(self.output_dir, filename)
            data = {
                'timestamps': self.timestamps[:self.sample_count],
                'lidar_data': self.lidar_data[:self.sample_count],
                'steering_data': self.steering_data[:self.sample_count],
                'speed_data': self.speed_data[:self.sample_count],
                'metadata': self.metadata_list[:self.sample_count]
            }
            with open(filepath, 'wb') as f:
                pickle.dump(data, f)
        
        self.get_logger().info(f'중간 저장 완료: {self.sample_count} 샘플')
    
    def finish_collection(self):
        """데이터 수집 완료"""
        if self.sample_count == 0 or self.lidar_data is None:
            self.get_logger().warn('수집된 데이터가 없습니다!')
            return
        
        # 최종 저장
        timestamp = int(time.time())
        
        if self.data_format == 'hdf5':
            filename = f"collected_data_speed_{self.target_speed}_final_{timestamp}.h5"
            filepath = os.path.join(self.output_dir, filename)
            self.save_data_hdf5(filepath, is_final=True)
        elif self.data_format == 'npz':
            filename = f"collected_data_speed_{self.target_speed}_final_{timestamp}.npz"
            filepath = os.path.join(self.output_dir, filename)
            self.save_data_npz(filepath)
        else:  # pickle
            filename = f"collected_data_speed_{self.target_speed}_final_{timestamp}.pkl"
            filepath = os.path.join(self.output_dir, filename)
            data = {
                'timestamps': self.timestamps[:self.sample_count],
                'lidar_data': self.lidar_data[:self.sample_count],
                'steering_data': self.steering_data[:self.sample_count],
                'speed_data': self.speed_data[:self.sample_count],
                'metadata': self.metadata_list[:self.sample_count]
            }
            with open(filepath, 'wb') as f:
                pickle.dump(data, f)
        
        # 통계 출력
        steering_angles = self.steering_data[:self.sample_count]
        
        self.get_logger().info('=== 데이터 수집 완료 ===')
        self.get_logger().info(f'총 샘플 수: {self.sample_count}')
        self.get_logger().info(f'평균 조향각: {np.mean(steering_angles):.3f}')
        self.get_logger().info(f'조향각 표준편차: {np.std(steering_angles):.3f}')
        self.get_logger().info(f'데이터 포맷: {self.data_format}')
        self.get_logger().info(f'압축 사용: {self.use_compression}')
        
        # 파일 크기 출력
        if os.path.exists(filepath):
            file_size_mb = os.path.getsize(filepath) / (1024 * 1024)
            self.get_logger().info(f'파일 크기: {file_size_mb:.2f} MB')
        
        self.get_logger().info(f'저장 파일: {filepath}')
        
        # 정지 명령
        stop_cmd = Float64MultiArray()
        stop_cmd.data = [0.0]
        self.pub_lw.publish(stop_cmd)
        self.pub_rw.publish(stop_cmd)
        self.pub_ls.publish(stop_cmd)
        self.pub_rs.publish(stop_cmd)
        
        # 노드 종료
        rclpy.shutdown()

def main(args=None):
    parser = argparse.ArgumentParser(description='Supervised Data Collector with Compression')
    parser.add_argument('--speed', type=float, default=40.0, help='Target speed for data collection')
    parser.add_argument('--duration', type=int, default=300, help='Collection duration in seconds')
    parser.add_argument('--output-dir', type=str, default='./collected_data', help='Output directory')
    parser.add_argument('--format', type=str, choices=['hdf5', 'npz', 'pickle'], default='hdf5', 
                       help='Data format (default: hdf5)')
    parser.add_argument('--no-compression', action='store_true', help='Disable compression')
    
    parsed_args = parser.parse_args()
    
    rclpy.init(args=args)
    
    collector = SupervisedDataCollector(
        target_speed=parsed_args.speed,
        collection_duration=parsed_args.duration,
        output_dir=parsed_args.output_dir,
        use_compression=not parsed_args.no_compression,
        data_format=parsed_args.format
    )
    
    try:
        rclpy.spin(collector)
    except KeyboardInterrupt:
        collector.get_logger().info('사용자에 의해 중단됨')
        collector.finish_collection()
    finally:
        if rclpy.ok():
            collector.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()