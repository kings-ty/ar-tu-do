#!/usr/bin/env python3
"""
F1 Supervised Learning Trainer

collected_data 폴더의 HDF5 파일들을 읽어서
CNN+LSTM 모델을 학습시킵니다.
"""

import os
import glob
import h5py
import json
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
from sklearn.model_selection import train_test_split
import matplotlib.pyplot as plt
from datetime import datetime
import argparse
from typing import List, Tuple, Dict

from f1_neural_network import F1DrivingNet, create_f1_model, F1NetworkUtils

class F1SequenceDataset(Dataset):
    """F1 시계열 데이터셋"""
    
    def __init__(self, sequences: List[np.ndarray], targets: List[Dict], 
                 sequence_length: int = 5):
        self.sequences = sequences
        self.targets = targets
        self.sequence_length = sequence_length
        
    def __len__(self):
        return len(self.sequences)
    
    def __getitem__(self, idx):
        sequence = self.sequences[idx]
        target = self.targets[idx]
        
        # LiDAR 시퀀스 (seq_len, lidar_points)
        lidar_sequence = torch.FloatTensor(sequence['lidar'])
        
        # 속도 시퀀스 (seq_len,)
        speed_sequence = torch.FloatTensor(sequence['speed']).unsqueeze(-1)
        
        # 타겟 (마지막 프레임의 결정)
        steering_target = torch.FloatTensor([target['steering']])
        speed_target = torch.FloatTensor([target['speed']])
        
        return lidar_sequence, speed_sequence, steering_target, speed_target

class F1DataLoader:
    """HDF5 데이터 로더 및 시계열 변환기"""
    
    def __init__(self, data_dir: str = "./collected_data", sequence_length: int = 5):
        self.data_dir = data_dir
        self.sequence_length = sequence_length
        
    def load_hdf5_file(self, filepath: str) -> Dict:
        """단일 HDF5 파일 로드"""
        print(f"Loading: {os.path.basename(filepath)}")
        
        with h5py.File(filepath, 'r') as f:
            data = {
                'timestamps': f['timestamps'][:],
                'lidar_data': f['lidar_data'][:],
                'steering_data': f['steering_data'][:],
                'speed_data': f['speed_data'][:],
                'metadata': json.loads(f['metadata'][()])
            }
            
            # 파일 정보
            sample_count = f.attrs.get('sample_count', len(data['timestamps']))
            lidar_points = f.attrs.get('lidar_points', data['lidar_data'].shape[1])
            
            print(f"  샘플 수: {sample_count}, LiDAR 포인트: {lidar_points}")
            
        return data
    
    def load_all_data(self) -> List[Dict]:
        """모든 HDF5 파일 로드"""
        if not os.path.exists(self.data_dir):
            raise FileNotFoundError(f"데이터 디렉토리가 없습니다: {self.data_dir}")
        
        # final 파일들 우선, 그 다음 temp 파일들
        final_files = glob.glob(os.path.join(self.data_dir, "*final*.h5"))
        temp_files = glob.glob(os.path.join(self.data_dir, "*temp*.h5"))
        
        all_files = sorted(final_files) + sorted(temp_files)
        
        if not all_files:
            raise FileNotFoundError("HDF5 파일이 없습니다!")
        
        print(f"=== 데이터 로딩: {len(all_files)}개 파일 ===")
        
        all_data = []
        for filepath in all_files:
            try:
                data = self.load_hdf5_file(filepath)
                all_data.append(data)
            except Exception as e:
                print(f"파일 로드 실패 {filepath}: {e}")
                continue
        
        return all_data
    
    def create_sequences(self, all_data: List[Dict]) -> Tuple[List, List]:
        """시계열 시퀀스 생성"""
        print(f"=== 시계열 시퀀스 생성 (길이: {self.sequence_length}) ===")
        
        sequences = []
        targets = []
        
        for data_idx, data in enumerate(all_data):
            lidar_data = data['lidar_data']
            steering_data = data['steering_data']
            speed_data = data['speed_data']
            timestamps = data['timestamps']
            
            # 최소 sequence_length 만큼의 데이터가 필요
            if len(lidar_data) < self.sequence_length:
                print(f"  데이터 {data_idx}: 길이 부족 ({len(lidar_data)} < {self.sequence_length})")
                continue
            
            # 슬라이딩 윈도우로 시퀀스 생성
            for i in range(len(lidar_data) - self.sequence_length + 1):
                # 입력 시퀀스 (처음 sequence_length-1 프레임)
                lidar_seq = lidar_data[i:i+self.sequence_length-1]  # shape: (4, lidar_points)
                speed_seq = speed_data[i:i+self.sequence_length-1]  # shape: (4,)
                
                # 타겟 (마지막 프레임의 결정)
                target_steering = steering_data[i+self.sequence_length-1]
                target_speed = speed_data[i+self.sequence_length-1]
                
                # 데이터 품질 체크
                if (np.isnan(lidar_seq).any() or np.isnan(speed_seq).any() or 
                    np.isnan(target_steering) or np.isnan(target_speed)):
                    continue
                
                # 극단적인 값 필터링
                if abs(target_steering) > 0.3 or target_speed < 10 or target_speed > 100:
                    continue
                
                sequences.append({
                    'lidar': lidar_seq,
                    'speed': speed_seq
                })
                
                targets.append({
                    'steering': target_steering,
                    'speed': target_speed / 100.0  # 0-1 범위로 정규화
                })
        
        print(f"생성된 시퀀스 수: {len(sequences)}")
        return sequences, targets
    
    def create_dataset(self, validation_split: float = 0.2) -> Tuple[F1SequenceDataset, F1SequenceDataset]:
        """학습/검증 데이터셋 생성"""
        # 모든 데이터 로드
        all_data = self.load_all_data()
        
        # 시계열 시퀀스 생성
        sequences, targets = self.create_sequences(all_data)
        
        if len(sequences) == 0:
            raise ValueError("생성된 시퀀스가 없습니다!")
        
        # 학습/검증 분할
        train_seq, val_seq, train_targets, val_targets = train_test_split(
            sequences, targets, test_size=validation_split, random_state=42
        )
        
        print(f"학습 시퀀스: {len(train_seq)}, 검증 시퀀스: {len(val_seq)}")
        
        # 데이터셋 생성
        train_dataset = F1SequenceDataset(train_seq, train_targets, self.sequence_length)
        val_dataset = F1SequenceDataset(val_seq, val_targets, self.sequence_length)
        
        return train_dataset, val_dataset

class F1Trainer:
    """F1 모델 학습기"""
    
    def __init__(self, model: F1DrivingNet, device: str = 'auto'):
        self.model = model
        self.device = torch.device('cuda' if torch.cuda.is_available() and device != 'cpu' else 'cpu')
        self.model.to(self.device)
        
        print(f"학습 디바이스: {self.device}")
        
        # 손실 함수
        self.steering_criterion = nn.MSELoss()
        self.speed_criterion = nn.MSELoss()
        
        # 옵티마이저
        self.optimizer = optim.Adam(self.model.parameters(), lr=0.001, weight_decay=1e-5)
        self.scheduler = optim.lr_scheduler.ReduceLROnPlateau(
            self.optimizer, mode='min', patience=10, factor=0.5
        )
        
        # 학습 기록
        self.train_losses = []
        self.val_losses = []
        self.steering_losses = []
        self.speed_losses = []
        
    def train_epoch(self, dataloader: DataLoader) -> float:
        """한 에포크 학습"""
        self.model.train()
        total_loss = 0.0
        steering_loss_sum = 0.0
        speed_loss_sum = 0.0
        
        for batch_idx, (lidar_seq, speed_seq, steering_target, speed_target) in enumerate(dataloader):
            lidar_seq = lidar_seq.to(self.device)
            speed_seq = speed_seq.to(self.device)
            steering_target = steering_target.to(self.device)
            speed_target = speed_target.to(self.device)
            
            # 순전파
            self.optimizer.zero_grad()
            steering_pred, speed_pred = self.model(lidar_seq, speed_seq)
            
            # 손실 계산
            steering_loss = self.steering_criterion(steering_pred, steering_target)
            speed_loss = self.speed_criterion(speed_pred, speed_target)
            
            # 가중 결합 (조향이 더 중요)
            total_batch_loss = steering_loss * 2.0 + speed_loss * 1.0
            
            # 역전파
            total_batch_loss.backward()
            
            # 그래디언트 클리핑
            torch.nn.utils.clip_grad_norm_(self.model.parameters(), max_norm=1.0)
            
            self.optimizer.step()
            
            total_loss += total_batch_loss.item()
            steering_loss_sum += steering_loss.item()
            speed_loss_sum += speed_loss.item()
        
        avg_loss = total_loss / len(dataloader)
        avg_steering_loss = steering_loss_sum / len(dataloader)
        avg_speed_loss = speed_loss_sum / len(dataloader)
        
        return avg_loss, avg_steering_loss, avg_speed_loss
    
    def validate(self, dataloader: DataLoader) -> float:
        """검증"""
        self.model.eval()
        total_loss = 0.0
        steering_loss_sum = 0.0
        speed_loss_sum = 0.0
        
        with torch.no_grad():
            for lidar_seq, speed_seq, steering_target, speed_target in dataloader:
                lidar_seq = lidar_seq.to(self.device)
                speed_seq = speed_seq.to(self.device)
                steering_target = steering_target.to(self.device)
                speed_target = speed_target.to(self.device)
                
                steering_pred, speed_pred = self.model(lidar_seq, speed_seq)
                
                steering_loss = self.steering_criterion(steering_pred, steering_target)
                speed_loss = self.speed_criterion(speed_pred, speed_target)
                
                total_batch_loss = steering_loss * 2.0 + speed_loss * 1.0
                
                total_loss += total_batch_loss.item()
                steering_loss_sum += steering_loss.item()
                speed_loss_sum += speed_loss.item()
        
        avg_loss = total_loss / len(dataloader)
        avg_steering_loss = steering_loss_sum / len(dataloader)
        avg_speed_loss = speed_loss_sum / len(dataloader)
        
        return avg_loss, avg_steering_loss, avg_speed_loss
    
    def train(self, train_dataset: F1SequenceDataset, val_dataset: F1SequenceDataset,
              epochs: int = 100, batch_size: int = 32) -> str:
        """전체 학습 프로세스"""
        
        # 데이터 로더
        train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True, num_workers=2)
        val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False, num_workers=2)
        
        print(f"=== 학습 시작: {epochs} 에포크, 배치 크기: {batch_size} ===")
        
        best_val_loss = float('inf')
        patience_counter = 0
        
        for epoch in range(epochs):
            # 학습
            train_loss, train_steering, train_speed = self.train_epoch(train_loader)
            
            # 검증
            val_loss, val_steering, val_speed = self.validate(val_loader)
            
            # 기록
            self.train_losses.append(train_loss)
            self.val_losses.append(val_loss)
            self.steering_losses.append(val_steering)
            self.speed_losses.append(val_speed)
            
            # 스케줄러 업데이트
            self.scheduler.step(val_loss)
            
            # 로그 출력
            if (epoch + 1) % 10 == 0 or epoch < 5:
                print(f"Epoch {epoch+1:3d}/{epochs} | "
                      f"Train: {train_loss:.4f} | Val: {val_loss:.4f} | "
                      f"Steering: {val_steering:.4f} | Speed: {val_speed:.4f}")
            
            # 조기 종료
            if val_loss < best_val_loss:
                best_val_loss = val_loss
                patience_counter = 0
                
                # 최고 모델 저장
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                best_model_path = f"f1_model_best_{timestamp}.pth"
                torch.save({
                    'model_state_dict': self.model.state_dict(),
                    'optimizer_state_dict': self.optimizer.state_dict(),
                    'epoch': epoch,
                    'val_loss': val_loss,
                    'train_losses': self.train_losses,
                    'val_losses': self.val_losses
                }, best_model_path)
            else:
                patience_counter += 1
                
            if patience_counter >= 20:
                print(f"조기 종료: {epoch+1} 에포크에서 중단")
                break
        
        # 최종 모델 저장
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        final_model_path = f"f1_model_final_{timestamp}.pth"
        torch.save({
            'model_state_dict': self.model.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
            'epoch': epochs,
            'val_loss': val_loss,
            'train_losses': self.train_losses,
            'val_losses': self.val_losses
        }, final_model_path)
        
        print(f"학습 완료! 모델 저장: {final_model_path}")
        
        # 학습 곡선 그래프
        self.plot_training_history(timestamp)
        
        return final_model_path
    
    def plot_training_history(self, timestamp: str):
        """학습 곡선 플롯"""
        plt.figure(figsize=(15, 5))
        
        # 전체 손실
        plt.subplot(1, 3, 1)
        plt.plot(self.train_losses, label='Train Loss')
        plt.plot(self.val_losses, label='Val Loss')
        plt.title('Total Loss')
        plt.xlabel('Epoch')
        plt.ylabel('Loss')
        plt.legend()
        plt.grid(True)
        
        # 조향각 손실
        plt.subplot(1, 3, 2)
        plt.plot(self.steering_losses, label='Steering Loss')
        plt.title('Steering Loss')
        plt.xlabel('Epoch')
        plt.ylabel('MSE')
        plt.legend()
        plt.grid(True)
        
        # 속도 손실
        plt.subplot(1, 3, 3)
        plt.plot(self.speed_losses, label='Speed Loss')
        plt.title('Speed Loss')
        plt.xlabel('Epoch')
        plt.ylabel('MSE')
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        plt.savefig(f'training_history_{timestamp}.png', dpi=150)
        plt.close()
        
        print(f"학습 곡선 저장: training_history_{timestamp}.png")

def main():
    parser = argparse.ArgumentParser(description='F1 Supervised Learning Trainer')
    parser.add_argument('--data-dir', type=str, default='./collected_data', 
                       help='Data directory')
    parser.add_argument('--epochs', type=int, default=100, help='Number of epochs')
    parser.add_argument('--batch-size', type=int, default=32, help='Batch size')
    parser.add_argument('--sequence-length', type=int, default=5, help='Sequence length')
    parser.add_argument('--device', type=str, default='auto', choices=['auto', 'cpu', 'cuda'])
    
    args = parser.parse_args()
    
    print("=== F1 Supervised Learning Trainer ===")
    
    try:
        # 데이터 로더
        data_loader = F1DataLoader(args.data_dir, args.sequence_length)
        train_dataset, val_dataset = data_loader.create_dataset()
        
        # 모델 생성
        model = create_f1_model(lidar_points=1080, sequence_length=args.sequence_length)
        F1NetworkUtils.model_summary(model)
        
        # 학습기
        trainer = F1Trainer(model, args.device)
        
        # 학습 실행
        model_path = trainer.train(train_dataset, val_dataset, 
                                 args.epochs, args.batch_size)
        
        print(f"✅ 학습 완료: {model_path}")
        
    except Exception as e:
        print(f"❌ 학습 실패: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()