#!/usr/bin/env python3

import json
import os
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from datetime import datetime

class CircuitDataset(Dataset):
    """서킷 주행 데이터셋"""
    def __init__(self, laser_data, drive_commands):
        self.laser_data = torch.FloatTensor(laser_data)
        self.drive_commands = torch.FloatTensor(drive_commands)
        
    def __len__(self):
        return len(self.laser_data)
        
    def __getitem__(self, idx):
        return self.laser_data[idx], self.drive_commands[idx]

class CircuitDrivingNet(nn.Module):
    """서킷 자율주행을 위한 신경망"""
    def __init__(self, input_size=360, hidden_size=128):
        super(CircuitDrivingNet, self).__init__()
        
        self.network = nn.Sequential(
            # 입력층 - LiDAR 데이터 처리
            nn.Linear(input_size, hidden_size),
            nn.ReLU(),
            nn.Dropout(0.2),
            
            # 히든층 1
            nn.Linear(hidden_size, hidden_size // 2),
            nn.ReLU(),
            nn.Dropout(0.2),
            
            # 히든층 2
            nn.Linear(hidden_size // 2, hidden_size // 4),
            nn.ReLU(),
            nn.Dropout(0.1),
            
            # 출력층 - 조향각과 속도
            nn.Linear(hidden_size // 4, 2),
            nn.Tanh()  # -1 ~ 1 범위로 출력
        )
        
    def forward(self, x):
        return self.network(x)

class MLTrainer:
    def __init__(self):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(f"사용 디바이스: {self.device}")
        
        # 데이터 디렉토리
        self.package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.data_dir = os.path.join(self.package_dir, 'data')
        self.model_dir = os.path.join(self.package_dir, 'models')
        os.makedirs(self.model_dir, exist_ok=True)
        
    def load_training_data(self):
        """훈련 데이터 로드"""
        if not os.path.exists(self.data_dir):
            raise Exception("데이터 디렉토리가 없습니다. 먼저 웨이포인트를 기록해주세요.")
            
        training_files = [f for f in os.listdir(self.data_dir) if f.startswith('training_data_')]
        
        if not training_files:
            raise Exception("훈련 데이터가 없습니다. 먼저 수동 주행을 기록해주세요.")
            
        all_laser_data = []
        all_drive_commands = []
        
        for file in training_files:
            file_path = os.path.join(self.data_dir, file)
            print(f"로딩 중: {file}")
            
            with open(file_path, 'r') as f:
                data = json.load(f)
                
            for sample in data:
                laser_scan = sample['laser_scan']
                drive_cmd = sample['drive_command']
                
                # LiDAR 데이터 전처리
                laser_scan = np.array(laser_scan)
                laser_scan = np.clip(laser_scan, 0, 10)  # 거리 제한
                laser_scan = laser_scan / 10.0  # 정규화
                
                # 드라이브 명령 전처리
                angle = drive_cmd['angle']
                velocity = drive_cmd['velocity']
                
                # 속도 정규화 (0-1 범위를 -1~1로)
                velocity_normalized = (velocity * 2) - 1
                
                all_laser_data.append(laser_scan)
                all_drive_commands.append([angle, velocity_normalized])
                
        print(f"총 {len(all_laser_data)}개의 훈련 샘플 로드됨")
        return np.array(all_laser_data), np.array(all_drive_commands)
    
    def train_model(self, epochs=100, batch_size=32, learning_rate=0.001):
        """모델 훈련"""
        print("훈련 데이터 로딩...")
        laser_data, drive_commands = self.load_training_data()
        
        # 데이터 분할
        X_train, X_test, y_train, y_test = train_test_split(
            laser_data, drive_commands, test_size=0.2, random_state=42
        )
        
        # 데이터셋 생성
        train_dataset = CircuitDataset(X_train, y_train)
        test_dataset = CircuitDataset(X_test, y_test)
        
        train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
        test_loader = DataLoader(test_dataset, batch_size=batch_size, shuffle=False)
        
        # 모델 생성
        input_size = laser_data.shape[1]
        model = CircuitDrivingNet(input_size=input_size).to(self.device)
        
        # 손실함수와 옵티마이저
        criterion = nn.MSELoss()
        optimizer = optim.Adam(model.parameters(), lr=learning_rate)
        scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, patience=10)
        
        # 훈련 기록
        train_losses = []
        test_losses = []
        
        print(f"모델 훈련 시작... (에포크: {epochs})")
        
        for epoch in range(epochs):
            # 훈련
            model.train()
            train_loss = 0.0
            for inputs, targets in train_loader:
                inputs, targets = inputs.to(self.device), targets.to(self.device)
                
                optimizer.zero_grad()
                outputs = model(inputs)
                loss = criterion(outputs, targets)
                loss.backward()
                optimizer.step()
                
                train_loss += loss.item()
            
            train_loss /= len(train_loader)
            
            # 검증
            model.eval()
            test_loss = 0.0
            with torch.no_grad():
                for inputs, targets in test_loader:
                    inputs, targets = inputs.to(self.device), targets.to(self.device)
                    outputs = model(inputs)
                    loss = criterion(outputs, targets)
                    test_loss += loss.item()
            
            test_loss /= len(test_loader)
            
            # 스케줄러 업데이트
            scheduler.step(test_loss)
            
            train_losses.append(train_loss)
            test_losses.append(test_loss)
            
            if (epoch + 1) % 10 == 0:
                print(f"에포크 {epoch+1}/{epochs} - 훈련 손실: {train_loss:.6f}, 검증 손실: {test_loss:.6f}")
        
        # 모델 저장
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        model_path = os.path.join(self.model_dir, f'circuit_model_{timestamp}.pth')
        
        torch.save({
            'model_state_dict': model.state_dict(),
            'input_size': input_size,
            'train_losses': train_losses,
            'test_losses': test_losses,
            'epochs': epochs
        }, model_path)
        
        print(f"모델 저장 완료: {model_path}")
        
        # 손실 그래프 저장
        self.plot_training_history(train_losses, test_losses, timestamp)
        
        return model, model_path
    
    def plot_training_history(self, train_losses, test_losses, timestamp):
        """훈련 과정 그래프 저장"""
        plt.figure(figsize=(10, 6))
        plt.plot(train_losses, label='훈련 손실')
        plt.plot(test_losses, label='검증 손실')
        plt.xlabel('에포크')
        plt.ylabel('손실')
        plt.title('모델 훈련 과정')
        plt.legend()
        plt.grid(True)
        
        plot_path = os.path.join(self.model_dir, f'training_history_{timestamp}.png')
        plt.savefig(plot_path)
        plt.close()
        
        print(f"훈련 그래프 저장: {plot_path}")

def main():
    trainer = MLTrainer()
    
    try:
        print("=== 서킷 자율주행 ML 모델 훈련 ===")
        model, model_path = trainer.train_model(epochs=200, batch_size=64)
        print("훈련 완료!")
        
    except Exception as e:
        print(f"훈련 중 오류 발생: {e}")

if __name__ == '__main__':
    main()