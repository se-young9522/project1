"""
테스트 스크립트 - 학습된 모델 평가 및 성능 지표 저장
"""
import torch
import numpy as np
from tqdm import tqdm
from pathlib import Path

# 로컬 모듈
import config
from model import get_model
from dataset import get_dataloaders
from utils import calculate_metrics, save_metrics_to_file, plot_confusion_matrix


def test_model(model, test_loader, device):
    """모델 테스트"""
    model.eval()
    
    all_predictions = []
    all_labels = []
    
    print("\n테스트 중...")
    with torch.no_grad():
        for images, labels in tqdm(test_loader, desc='Testing'):
            images = images.to(device)
            labels = labels.to(device)
            
            # Forward
            outputs = model(images)
            _, predicted = torch.max(outputs, 1)
            
            # 결과 저장
            all_predictions.extend(predicted.cpu().numpy())
            all_labels.extend(labels.cpu().numpy())
    
    return np.array(all_predictions), np.array(all_labels)


def main():
    """메인 테스트 함수"""
    
    print("=" * 80)
    print("행동 인식 모델 테스트")
    print("=" * 80)
    
    # 디바이스 설정
    device = torch.device(config.DEVICE if torch.cuda.is_available() else 'cpu')
    print(f"\n디바이스: {device}")
    
    # 데이터 로더 생성
    print("\n데이터 로더 생성 중...")
    _, _, test_loader, class_to_idx = get_dataloaders(
        config.DATA_DIR,
        batch_size=config.BATCH_SIZE,
        num_workers=config.NUM_WORKERS,
        input_size=config.INPUT_SIZE
    )
    
    # 클래스 이름 (인덱스 순서대로)
    idx_to_class = {v: k for k, v in class_to_idx.items()}
    class_names = [idx_to_class[i] for i in range(len(idx_to_class))]
    
    print(f"테스트 배치 수: {len(test_loader)}")
    print(f"클래스: {class_names}")
    
    # 모델 로드
    checkpoint_path = config.CHECKPOINT_DIR / 'best_model.pth'
    
    if not checkpoint_path.exists():
        print(f"\n오류: 체크포인트를 찾을 수 없습니다: {checkpoint_path}")
        print("먼저 train.py를 실행하여 모델을 학습하세요.")
        return
    
    print(f"\n모델 로드 중: {checkpoint_path}")
    
    # 모델 생성
    model = get_model(
        num_classes=config.NUM_CLASSES,
        pretrained=False,  # 학습된 가중치 사용
        device=device
    )
    
    # 체크포인트 로드
    checkpoint = torch.load(checkpoint_path, map_location=device)
    model.load_state_dict(checkpoint['model_state_dict'])
    
    print(f"✓ 모델 로드 완료 (Epoch {checkpoint['epoch']+1}, Val Acc: {checkpoint['val_acc']:.4f})")
    
    # 테스트 실행
    predictions, labels = test_model(model, test_loader, device)
    
    # 성능 지표 계산
    print("\n성능 지표 계산 중...")
    metrics = calculate_metrics(labels, predictions, class_names)
    
    # 결과 출력
    print("\n" + "=" * 80)
    print("테스트 결과")
    print("=" * 80)
    print(f"\n전체 정확도: {metrics['accuracy']:.4f}")
    print(f"\n평균 Precision: {metrics['avg_precision']:.4f}")
    print(f"평균 Recall:    {metrics['avg_recall']:.4f}")
    print(f"평균 F1-Score:  {metrics['avg_f1']:.4f}")
    
    print("\n클래스별 성능:")
    print("-" * 80)
    print(f"{'클래스':<15} {'Precision':<12} {'Recall':<12} {'F1-Score':<12}")
    print("-" * 80)
    for i, class_name in enumerate(class_names):
        print(f"{class_name:<15} "
              f"{metrics['precision'][i]:<12.4f} "
              f"{metrics['recall'][i]:<12.4f} "
              f"{metrics['f1_score'][i]:<12.4f}")
    
    # 결과를 txt 파일로 저장
    results_txt_path = config.RESULTS_DIR / 'test_results.txt'
    save_metrics_to_file(metrics, class_names, results_txt_path)
    
    # Confusion Matrix 시각화 및 저장
    cm_plot_path = config.RESULTS_DIR / 'confusion_matrix.png'
    plot_confusion_matrix(metrics['confusion_matrix'], class_names, cm_plot_path)
    
    print("\n" + "=" * 80)
    print("테스트 완료!")
    print("=" * 80)
    print(f"✓ 결과 저장: {results_txt_path}")
    print(f"✓ Confusion Matrix 저장: {cm_plot_path}")


if __name__ == '__main__':
    main()
