"""
학습 스크립트 - 행동 인식 모델 학습
"""
import torch
import torch.nn as nn
import torch.optim as optim
from torch.optim.lr_scheduler import ReduceLROnPlateau
import numpy as np
from tqdm import tqdm
import time
from pathlib import Path

# 로컬 모듈
import config
from model import get_model
from dataset import get_dataloaders
from utils import AverageMeter, plot_training_history

# 시드 설정
torch.manual_seed(config.SEED)
np.random.seed(config.SEED)
if torch.cuda.is_available():
    torch.cuda.manual_seed(config.SEED)


def train_one_epoch(model, train_loader, criterion, optimizer, device, epoch):
    """한 에포크 학습"""
    model.train()
    
    losses = AverageMeter()
    accuracies = AverageMeter()
    
    pbar = tqdm(train_loader, desc=f'Epoch {epoch+1}/{config.NUM_EPOCHS} [Train]')
    
    for images, labels in pbar:
        images = images.to(device)
        labels = labels.to(device)
        
        # Forward
        outputs = model(images)
        loss = criterion(outputs, labels)
        
        # Backward
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        
        # 정확도 계산
        _, predicted = torch.max(outputs, 1)
        accuracy = (predicted == labels).float().mean()
        
        # 메트릭 업데이트
        losses.update(loss.item(), images.size(0))
        accuracies.update(accuracy.item(), images.size(0))
        
        # 진행 상황 표시
        pbar.set_postfix({
            'loss': f'{losses.avg:.4f}',
            'acc': f'{accuracies.avg:.4f}'
        })
    
    return losses.avg, accuracies.avg


def validate(model, val_loader, criterion, device, epoch):
    """검증"""
    model.eval()
    
    losses = AverageMeter()
    accuracies = AverageMeter()
    
    pbar = tqdm(val_loader, desc=f'Epoch {epoch+1}/{config.NUM_EPOCHS} [Val]  ')
    
    with torch.no_grad():
        for images, labels in pbar:
            images = images.to(device)
            labels = labels.to(device)
            
            # Forward
            outputs = model(images)
            loss = criterion(outputs, labels)
            
            # 정확도 계산
            _, predicted = torch.max(outputs, 1)
            accuracy = (predicted == labels).float().mean()
            
            # 메트릭 업데이트
            losses.update(loss.item(), images.size(0))
            accuracies.update(accuracy.item(), images.size(0))
            
            # 진행 상황 표시
            pbar.set_postfix({
                'loss': f'{losses.avg:.4f}',
                'acc': f'{accuracies.avg:.4f}'
            })
    
    return losses.avg, accuracies.avg


def main():
    """메인 학습 함수"""
    
    print("=" * 80)
    print("행동 인식 모델 학습 시작")
    print("=" * 80)
    
    # 디바이스 설정
    device = torch.device(config.DEVICE if torch.cuda.is_available() else 'cpu')
    print(f"\n디바이스: {device}")
    
    # 데이터 로더 생성
    print("\n데이터 로더 생성 중...")
    train_loader, val_loader, test_loader, class_to_idx = get_dataloaders(
        config.DATA_DIR,
        batch_size=config.BATCH_SIZE,
        num_workers=config.NUM_WORKERS,
        input_size=config.INPUT_SIZE
    )
    
    print(f"학습 배치 수: {len(train_loader)}")
    print(f"검증 배치 수: {len(val_loader)}")
    print(f"테스트 배치 수: {len(test_loader)}")
    
    # 모델 생성
    print(f"\n모델 생성 중... ({config.MODEL_NAME})")
    model = get_model(
        num_classes=config.NUM_CLASSES,
        pretrained=config.PRETRAINED,
        device=device
    )
    
    # Loss 함수 및 Optimizer
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(
        model.parameters(),
        lr=config.LEARNING_RATE,
        weight_decay=config.WEIGHT_DECAY
    )
    
    # Learning Rate Scheduler
    scheduler = ReduceLROnPlateau(
        optimizer,
        mode='min',
        factor=0.5,
        patience=5
    )
    
    # 학습 히스토리
    history = {
        'train_loss': [],
        'train_acc': [],
        'val_loss': [],
        'val_acc': []
    }
    
    # 최고 성능 추적
    best_val_acc = 0.0
    best_epoch = 0
    patience_counter = 0
    
    print("\n" + "=" * 80)
    print("학습 시작")
    print("=" * 80)
    
    start_time = time.time()
    
    for epoch in range(config.NUM_EPOCHS):
        # 학습
        train_loss, train_acc = train_one_epoch(
            model, train_loader, criterion, optimizer, device, epoch
        )
        
        # 검증
        val_loss, val_acc = validate(
            model, val_loader, criterion, device, epoch
        )
        
        # 히스토리 업데이트
        history['train_loss'].append(train_loss)
        history['train_acc'].append(train_acc)
        history['val_loss'].append(val_loss)
        history['val_acc'].append(val_acc)
        
        # Learning Rate 조정
        scheduler.step(val_loss)
        
        # 결과 출력
        print(f"\nEpoch {epoch+1}/{config.NUM_EPOCHS} 요약:")
        print(f"  Train Loss: {train_loss:.4f}, Train Acc: {train_acc:.4f}")
        print(f"  Val Loss:   {val_loss:.4f}, Val Acc:   {val_acc:.4f}")
        
        # 최고 성능 모델 저장
        if val_acc > best_val_acc:
            best_val_acc = val_acc
            best_epoch = epoch + 1
            patience_counter = 0
            
            # 체크포인트 저장
            checkpoint_path = config.CHECKPOINT_DIR / 'best_model.pth'
            torch.save({
                'epoch': epoch,
                'model_state_dict': model.state_dict(),
                'optimizer_state_dict': optimizer.state_dict(),
                'val_acc': val_acc,
                'val_loss': val_loss,
                'class_to_idx': class_to_idx
            }, checkpoint_path)
            
            print(f"  ✓ 최고 성능 모델 저장! (Val Acc: {val_acc:.4f})")
        else:
            patience_counter += 1
            print(f"  성능 개선 없음 ({patience_counter}/{config.PATIENCE})")
        
        # Early Stopping
        if patience_counter >= config.PATIENCE:
            print(f"\nEarly Stopping! {config.PATIENCE} 에포크 동안 개선 없음.")
            break
        
        print("-" * 80)
    
    # 학습 완료
    elapsed_time = time.time() - start_time
    print("\n" + "=" * 80)
    print("학습 완료!")
    print("=" * 80)
    print(f"총 학습 시간: {elapsed_time/60:.2f}분")
    print(f"최고 검증 정확도: {best_val_acc:.4f} (Epoch {best_epoch})")
    
    # 학습 곡선 저장
    plot_path = config.RESULTS_DIR / 'training_history.png'
    plot_training_history(history, plot_path)
    
    # 마지막 모델도 저장
    last_checkpoint_path = config.CHECKPOINT_DIR / 'last_model.pth'
    torch.save({
        'epoch': epoch,
        'model_state_dict': model.state_dict(),
        'optimizer_state_dict': optimizer.state_dict(),
        'val_acc': val_acc,
        'val_loss': val_loss,
        'class_to_idx': class_to_idx
    }, last_checkpoint_path)
    
    print(f"\n✓ 마지막 모델 저장: {last_checkpoint_path}")
    print(f"✓ 최고 모델 저장: {config.CHECKPOINT_DIR / 'best_model.pth'}")
    print(f"✓ 학습 곡선 저장: {plot_path}")


if __name__ == '__main__':
    main()
