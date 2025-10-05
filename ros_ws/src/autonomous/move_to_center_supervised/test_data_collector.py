#!/usr/bin/env python3
"""
데이터 수집기 테스트 스크립트
실제 수집 전에 간단히 동작을 확인할 수 있습니다.
"""
import pickle
import os
import numpy as np
import matplotlib.pyplot as plt

def analyze_collected_data(data_file):
    """수집된 데이터 분석"""
    print(f"=== 데이터 분석: {data_file} ===")
    
    with open(data_file, 'rb') as f:
        data = pickle.load(f)
    
    if len(data) == 0:
        print("데이터가 없습니다!")
        return
    
    # 기본 통계
    steering_angles = [sample['target_steering'] for sample in data]
    speeds = [sample['target_speed'] for sample in data]
    
    print(f"총 샘플 수: {len(data)}")
    print(f"조향각 - 평균: {np.mean(steering_angles):.3f}, 표준편차: {np.std(steering_angles):.3f}")
    print(f"조향각 - 최소: {np.min(steering_angles):.3f}, 최대: {np.max(steering_angles):.3f}")
    print(f"속도 - 평균: {np.mean(speeds):.1f}, 표준편차: {np.std(speeds):.1f}")
    
    # 조향각 분포
    left_turns = sum(1 for angle in steering_angles if angle < -0.05)
    straight = sum(1 for angle in steering_angles if -0.05 <= angle <= 0.05)
    right_turns = sum(1 for angle in steering_angles if angle > 0.05)
    
    print(f"주행 패턴:")
    print(f"  좌회전: {left_turns} ({left_turns/len(data)*100:.1f}%)")
    print(f"  직진: {straight} ({straight/len(data)*100:.1f}%)")
    print(f"  우회전: {right_turns} ({right_turns/len(data)*100:.1f}%)")
    
    # 샘플 데이터 확인
    sample = data[0]
    print(f"\n샘플 데이터 구조:")
    print(f"  LiDAR 포인트 수: {len(sample['lidar_scan'])}")
    print(f"  메타데이터 키: {list(sample['metadata'].keys())}")
    
    return data

def plot_steering_timeline(data):
    """조향각 시계열 플롯"""
    if len(data) == 0:
        return
    
    timestamps = [sample['timestamp'] for sample in data]
    steering_angles = [sample['target_steering'] for sample in data]
    
    # 시작 시간을 0으로 정규화
    start_time = timestamps[0]
    normalized_times = [(t - start_time) for t in timestamps]
    
    plt.figure(figsize=(12, 6))
    plt.plot(normalized_times, steering_angles, 'b-', alpha=0.7, linewidth=1)
    plt.axhline(y=0, color='r', linestyle='--', alpha=0.5)
    plt.xlabel('시간 (초)')
    plt.ylabel('조향각 (라디안)')
    plt.title('조향각 시계열 데이터')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('steering_timeline.png', dpi=150)
    plt.show()
    print("조향각 그래프 저장: steering_timeline.png")

def check_data_quality(data):
    """데이터 품질 검사"""
    print(f"\n=== 데이터 품질 검사 ===")
    
    if len(data) == 0:
        return
    
    # 1. 급격한 조향 변화 체크
    steering_angles = [sample['target_steering'] for sample in data]
    steering_changes = [abs(steering_angles[i] - steering_angles[i-1]) 
                       for i in range(1, len(steering_angles))]
    
    large_changes = sum(1 for change in steering_changes if change > 0.2)
    print(f"급격한 조향 변화 (>0.2): {large_changes} ({large_changes/len(steering_changes)*100:.1f}%)")
    
    # 2. 벽 거리 분포
    left_distances = [sample['metadata']['left_distance'] for sample in data]
    right_distances = [sample['metadata']['right_distance'] for sample in data]
    
    print(f"좌측 벽 거리 - 평균: {np.mean(left_distances):.3f}m, 범위: {np.min(left_distances):.3f}-{np.max(left_distances):.3f}m")
    print(f"우측 벽 거리 - 평균: {np.mean(right_distances):.3f}m, 범위: {np.min(right_distances):.3f}-{np.max(right_distances):.3f}m")
    
    # 3. 위치 오차 분포
    position_errors = [sample['metadata']['position_error'] for sample in data]
    print(f"위치 오차 - 평균: {np.mean(position_errors):.3f}m, 표준편차: {np.std(position_errors):.3f}m")

def main():
    print("=== 데이터 수집기 테스트 도구 ===")
    
    # collected_data 디렉토리에서 최신 파일 찾기
    data_dir = "./collected_data"
    if not os.path.exists(data_dir):
        print(f"데이터 디렉토리가 없습니다: {data_dir}")
        print("먼저 데이터를 수집해주세요:")
        print("python supervised_data_collector.py --speed 40 --duration 60")
        return
    
    # .pkl 파일 찾기
    pkl_files = [f for f in os.listdir(data_dir) if f.endswith('.pkl')]
    
    if not pkl_files:
        print("수집된 데이터 파일이 없습니다.")
        return
    
    # 최신 파일 사용
    latest_file = sorted(pkl_files)[-1]
    data_path = os.path.join(data_dir, latest_file)
    
    print(f"분석할 파일: {latest_file}")
    
    # 데이터 분석
    data = analyze_collected_data(data_path)
    
    if data:
        check_data_quality(data)
        
        # 그래프 생성 여부 묻기
        try:
            response = input("\n조향각 그래프를 생성하시겠습니까? (y/n): ").lower()
            if response == 'y':
                plot_steering_timeline(data)
        except:
            pass  # 입력 없이 종료되는 경우

if __name__ == '__main__':
    main()