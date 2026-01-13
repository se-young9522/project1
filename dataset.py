"""
데이터셋 클래스 - 프레임 이미지를 로드하는 PyTorch Dataset
"""
import torch
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms
from PIL import Image
from pathlib import Path
import random

class ActionDataset(Dataset):
    """행동 인식 데이터셋"""
    
    def __init__(self, data_dir, split='train', transform=None, max_frames_per_video=30):
        """
        Args:
            data_dir: 데이터 디렉토리 경로
            split: 'train', 'validation', 'test'
            transform: 이미지 변환
            max_frames_per_video: 비디오당 최대 프레임 수 (메모리 절약)
        """
        self.data_dir = Path(data_dir)
        self.split = split
        self.transform = transform
        self.max_frames_per_video = max_frames_per_video
        
        # 데이터 수집
        self.samples = []
        self.class_to_idx = {}
        
        split_dir = self.data_dir / split
        
        # 각 행동 클래스 순회
        for idx, action_class in enumerate(sorted(split_dir.iterdir())):
            if not action_class.is_dir():
                continue
            
            self.class_to_idx[action_class.name] = idx
            
            # 각 비디오 폴더 순회
            for video_dir in action_class.iterdir():
                if not video_dir.is_dir():
                    continue
                
                frames_dir = video_dir / 'frames'
                if not frames_dir.exists():
                    continue
                
                # 프레임 이미지 수집
                frame_files = sorted(frames_dir.glob('*.jpg'))
                
                # 프레임이 너무 많으면 샘플링
                if len(frame_files) > self.max_frames_per_video:
                    # 균등하게 샘플링
                    indices = torch.linspace(0, len(frame_files)-1, self.max_frames_per_video).long()
                    frame_files = [frame_files[i] for i in indices]
                
                # 각 프레임을 개별 샘플로 추가
                for frame_path in frame_files:
                    self.samples.append({
                        'image_path': frame_path,
                        'label': idx,
                        'class_name': action_class.name
                    })
        
        print(f"{split} 데이터셋: {len(self.samples)}개 샘플, {len(self.class_to_idx)}개 클래스")
    
    def __len__(self):
        return len(self.samples)
    
    def __getitem__(self, idx):
        sample = self.samples[idx]
        
        # 이미지 로드
        image = Image.open(sample['image_path']).convert('RGB')
        
        # 변환 적용
        if self.transform:
            image = self.transform(image)
        
        label = sample['label']
        
        return image, label


def get_transforms(split='train', input_size=224):
    """데이터 증강 및 전처리"""
    
    if split == 'train':
        # 학습용: 데이터 증강 적용
        transform = transforms.Compose([
            transforms.Resize((input_size, input_size)),
            transforms.RandomHorizontalFlip(p=0.5),
            transforms.RandomRotation(10),
            transforms.ColorJitter(brightness=0.2, contrast=0.2, saturation=0.2),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], 
                               std=[0.229, 0.224, 0.225])
        ])
    else:
        # 검증/테스트용: 증강 없음
        transform = transforms.Compose([
            transforms.Resize((input_size, input_size)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], 
                               std=[0.229, 0.224, 0.225])
        ])
    
    return transform


def get_dataloaders(data_dir, batch_size=32, num_workers=0, input_size=224):
    """데이터 로더 생성"""
    
    # 데이터셋 생성
    train_dataset = ActionDataset(
        data_dir, 
        split='train',
        transform=get_transforms('train', input_size)
    )
    
    val_dataset = ActionDataset(
        data_dir,
        split='validation',
        transform=get_transforms('val', input_size)
    )
    
    test_dataset = ActionDataset(
        data_dir,
        split='test',
        transform=get_transforms('test', input_size)
    )
    
    # 데이터 로더 생성
    train_loader = DataLoader(
        train_dataset,
        batch_size=batch_size,
        shuffle=True,
        num_workers=num_workers,
        pin_memory=True
    )
    
    val_loader = DataLoader(
        val_dataset,
        batch_size=batch_size,
        shuffle=False,
        num_workers=num_workers,
        pin_memory=True
    )
    
    test_loader = DataLoader(
        test_dataset,
        batch_size=batch_size,
        shuffle=False,
        num_workers=num_workers,
        pin_memory=True
    )
    
    return train_loader, val_loader, test_loader, train_dataset.class_to_idx
