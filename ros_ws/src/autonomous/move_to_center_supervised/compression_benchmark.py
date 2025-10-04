#!/usr/bin/env python3
"""
데이터 압축 성능 벤치마크
다양한 포맷과 압축 방식의 크기 및 속도를 비교합니다.
"""
import numpy as np
import pickle
import h5py
import json
import time
import os
import gzip
import bz2
import lzma
from collections import defaultdict

def generate_sample_data(num_samples=1000):
    """샘플 데이터 생성 (실제 수집 데이터와 유사)"""
    print(f"샘플 데이터 생성 중: {num_samples} 샘플")
    
    # LiDAR 데이터 (360 포인트, 노이즈 포함)
    lidar_data = np.random.rand(num_samples, 360).astype(np.float32)
    lidar_data += np.random.normal(0, 0.1, (num_samples, 360))  # 노이즈
    lidar_data = np.clip(lidar_data, 0, 1)
    
    # 조향각 (부드러운 변화)
    steering_data = np.zeros(num_samples, dtype=np.float32)
    for i in range(1, num_samples):
        steering_data[i] = steering_data[i-1] + np.random.normal(0, 0.05)
        steering_data[i] = np.clip(steering_data[i], -0.3, 0.3)
    
    # 속도 (거의 일정)
    speed_data = np.full(num_samples, 40.0, dtype=np.float32)
    speed_data += np.random.normal(0, 1, num_samples)
    
    # 타임스탬프
    timestamps = np.linspace(0, num_samples/10, num_samples)
    
    # 메타데이터
    metadata_list = []
    for i in range(num_samples):
        metadata_list.append({
            'left_distance': float(np.random.uniform(0.5, 2.0)),
            'right_distance': float(np.random.uniform(0.5, 2.0)),
            'position_error': float(np.random.normal(0, 0.1)),
            'left_steer': float(steering_data[i] * 0.8),
            'right_steer': float(steering_data[i] * 0.8)
        })
    
    return {
        'timestamps': timestamps,
        'lidar_data': lidar_data,
        'steering_data': steering_data,
        'speed_data': speed_data,
        'metadata': metadata_list
    }

def benchmark_formats(data, test_dir="./compression_test"):
    """다양한 포맷 벤치마크"""
    os.makedirs(test_dir, exist_ok=True)
    results = {}
    
    print("=== 포맷별 크기 및 속도 비교 ===")
    
    # 1. Pickle (기본)
    print("1. Pickle (기본)")
    filepath = os.path.join(test_dir, "data.pkl")
    start_time = time.time()
    with open(filepath, 'wb') as f:
        pickle.dump(data, f)
    save_time = time.time() - start_time
    file_size = os.path.getsize(filepath)
    
    start_time = time.time()
    with open(filepath, 'rb') as f:
        loaded = pickle.load(f)
    load_time = time.time() - start_time
    
    results['pickle'] = {'size': file_size, 'save_time': save_time, 'load_time': load_time}
    print(f"   크기: {file_size/1024/1024:.2f} MB, 저장: {save_time:.3f}s, 로드: {load_time:.3f}s")
    
    # 2. Pickle + gzip
    print("2. Pickle + gzip")
    filepath = os.path.join(test_dir, "data.pkl.gz")
    start_time = time.time()
    with gzip.open(filepath, 'wb') as f:
        pickle.dump(data, f)
    save_time = time.time() - start_time
    file_size = os.path.getsize(filepath)
    
    start_time = time.time()
    with gzip.open(filepath, 'rb') as f:
        loaded = pickle.load(f)
    load_time = time.time() - start_time
    
    results['pickle_gzip'] = {'size': file_size, 'save_time': save_time, 'load_time': load_time}
    print(f"   크기: {file_size/1024/1024:.2f} MB, 저장: {save_time:.3f}s, 로드: {load_time:.3f}s")
    
    # 3. NPZ (압축)
    print("3. NumPy NPZ (압축)")
    filepath = os.path.join(test_dir, "data.npz")
    start_time = time.time()
    np.savez_compressed(filepath,
                       timestamps=data['timestamps'],
                       lidar_data=data['lidar_data'],
                       steering_data=data['steering_data'],
                       speed_data=data['speed_data'],
                       metadata=data['metadata'])
    save_time = time.time() - start_time
    file_size = os.path.getsize(filepath)
    
    start_time = time.time()
    loaded = np.load(filepath, allow_pickle=True)
    load_time = time.time() - start_time
    
    results['npz'] = {'size': file_size, 'save_time': save_time, 'load_time': load_time}
    print(f"   크기: {file_size/1024/1024:.2f} MB, 저장: {save_time:.3f}s, 로드: {load_time:.3f}s")
    
    # 4. HDF5 (압축 없음)
    print("4. HDF5 (압축 없음)")
    filepath = os.path.join(test_dir, "data.h5")
    start_time = time.time()
    with h5py.File(filepath, 'w') as f:
        f.create_dataset('timestamps', data=data['timestamps'])
        f.create_dataset('lidar_data', data=data['lidar_data'])
        f.create_dataset('steering_data', data=data['steering_data'])
        f.create_dataset('speed_data', data=data['speed_data'])
        f.create_dataset('metadata', data=json.dumps(data['metadata']))
    save_time = time.time() - start_time
    file_size = os.path.getsize(filepath)
    
    start_time = time.time()
    with h5py.File(filepath, 'r') as f:
        loaded_timestamps = f['timestamps'][:]
        loaded_lidar = f['lidar_data'][:]
        loaded_steering = f['steering_data'][:]
        loaded_speed = f['speed_data'][:]
        loaded_metadata = json.loads(f['metadata'][()])
    load_time = time.time() - start_time
    
    results['hdf5'] = {'size': file_size, 'save_time': save_time, 'load_time': load_time}
    print(f"   크기: {file_size/1024/1024:.2f} MB, 저장: {save_time:.3f}s, 로드: {load_time:.3f}s")
    
    # 5. HDF5 + gzip
    print("5. HDF5 + gzip")
    filepath = os.path.join(test_dir, "data_compressed.h5")
    start_time = time.time()
    with h5py.File(filepath, 'w') as f:
        f.create_dataset('timestamps', data=data['timestamps'], compression='gzip', compression_opts=9)
        f.create_dataset('lidar_data', data=data['lidar_data'], compression='gzip', compression_opts=9)
        f.create_dataset('steering_data', data=data['steering_data'], compression='gzip', compression_opts=9)
        f.create_dataset('speed_data', data=data['speed_data'], compression='gzip', compression_opts=9)
        f.create_dataset('metadata', data=json.dumps(data['metadata']), compression='gzip')
    save_time = time.time() - start_time
    file_size = os.path.getsize(filepath)
    
    start_time = time.time()
    with h5py.File(filepath, 'r') as f:
        loaded_timestamps = f['timestamps'][:]
        loaded_lidar = f['lidar_data'][:]
        loaded_steering = f['steering_data'][:]
        loaded_speed = f['speed_data'][:]
        loaded_metadata = json.loads(f['metadata'][()])
    load_time = time.time() - start_time
    
    results['hdf5_gzip'] = {'size': file_size, 'save_time': save_time, 'load_time': load_time}
    print(f"   크기: {file_size/1024/1024:.2f} MB, 저장: {save_time:.3f}s, 로드: {load_time:.3f}s")
    
    # 6. Float16 최적화 (HDF5 + gzip)
    print("6. HDF5 + gzip + Float16")
    filepath = os.path.join(test_dir, "data_float16.h5")
    start_time = time.time()
    with h5py.File(filepath, 'w') as f:
        f.create_dataset('timestamps', data=data['timestamps'].astype(np.float32), compression='gzip', compression_opts=9)
        f.create_dataset('lidar_data', data=data['lidar_data'].astype(np.float16), compression='gzip', compression_opts=9)
        f.create_dataset('steering_data', data=data['steering_data'].astype(np.float16), compression='gzip', compression_opts=9)
        f.create_dataset('speed_data', data=data['speed_data'].astype(np.float16), compression='gzip', compression_opts=9)
        f.create_dataset('metadata', data=json.dumps(data['metadata']), compression='gzip')
    save_time = time.time() - start_time
    file_size = os.path.getsize(filepath)
    
    results['hdf5_float16'] = {'size': file_size, 'save_time': save_time, 'load_time': 0}
    print(f"   크기: {file_size/1024/1024:.2f} MB, 저장: {save_time:.3f}s")
    
    return results

def print_summary(results):
    """결과 요약"""
    print("\n=== 요약 ===")
    print(f"{'포맷':<15} {'크기(MB)':<10} {'저장(s)':<10} {'로드(s)':<10} {'압축비':<10}")
    print("-" * 60)
    
    baseline_size = results['pickle']['size']
    
    for format_name, result in results.items():
        size_mb = result['size'] / 1024 / 1024
        compression_ratio = baseline_size / result['size']
        save_time = result['save_time']
        load_time = result.get('load_time', 0)
        
        print(f"{format_name:<15} {size_mb:<10.2f} {save_time:<10.3f} {load_time:<10.3f} {compression_ratio:<10.1f}x")
    
    # 추천
    print("\n=== 추천 ===")
    best_compression = min(results.items(), key=lambda x: x[1]['size'])
    fastest_save = min(results.items(), key=lambda x: x[1]['save_time'])
    
    print(f"최고 압축: {best_compression[0]} ({best_compression[1]['size']/1024/1024:.2f} MB)")
    print(f"가장 빠른 저장: {fastest_save[0]} ({fastest_save[1]['save_time']:.3f}s)")
    print(f"권장: hdf5_gzip (압축률과 속도의 균형)")

def main():
    print("=== 데이터 압축 벤치마크 ===")
    
    # 테스트 크기들
    test_sizes = [100, 1000, 5000]
    
    for size in test_sizes:
        print(f"\n{'='*50}")
        print(f"테스트 크기: {size} 샘플")
        print(f"{'='*50}")
        
        # 샘플 데이터 생성
        data = generate_sample_data(size)
        
        # 벤치마크 실행
        results = benchmark_formats(data, f"./compression_test_{size}")
        
        # 결과 출력
        print_summary(results)
    
    print(f"\n{'='*50}")
    print("결론:")
    print("- 최고 압축: HDF5 + gzip + Float16")
    print("- 권장: HDF5 + gzip (Float32)")
    print("- 빠른 처리: NPZ")
    print("- 호환성: Pickle + gzip")

if __name__ == '__main__':
    main()