"""
자동 라벨링 스크립트 (Segmentation, 점진적 확장)

사용법:
  python auto_label_seg.py                 # 기본: 200장씩 처리
  python auto_label_seg.py --batch 100     # 100장씩 처리
  python auto_label_seg.py --conf 0.4      # 신뢰도 임계값 변경

흐름:
  1. labels_v2에 없는 이미지만 대상
  2. --batch 장수만큼 추론 후 저장
  3. 저장된 라벨을 CVAT/Roboflow에서 검수
  4. 검수 완료 후 train_seg.py 재실행
"""

import argparse
import os
from ultralytics import YOLO

# ─── 경로 설정 ─────────────────────────────────────────
IMAGE_DIR = r'C:\Users\ASUS\Desktop\datasets\videos\images'
LABEL_DIR = r'C:\Users\ASUS\Desktop\datasets\videos\labels'
MODEL_PATH = r'C:\Users\ASUS\Desktop\samsung wellstory\runs\segment\bowl_seg_v37\weights\best.pt'
# ────────────────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--batch', type=int, default=200,
                        help='이번 실행에서 처리할 이미지 수 (기본: 200)')
    parser.add_argument('--conf', type=float, default=0.5,
                        help='신뢰도 임계값 (기본: 0.5)')
    parser.add_argument('--model', type=str, default=MODEL_PATH,
                        help='사용할 모델 경로')
    return parser.parse_args()

def main():
    args = parse_args()
    os.makedirs(LABEL_DIR, exist_ok=True)

    # 아직 라벨이 없는 이미지 목록
    all_images = sorted(f for f in os.listdir(IMAGE_DIR) if f.endswith('.jpg'))
    unlabeled = [
        f for f in all_images
        if not os.path.exists(os.path.join(LABEL_DIR, f.replace('.jpg', '.txt')))
    ]

    total_remaining = len(unlabeled)
    batch = unlabeled[:args.batch]  # 이번 회차에 처리할 이미지

    print(f"전체 미라벨 이미지: {total_remaining}장")
    print(f"이번 처리 대상:     {len(batch)}장 (--batch {args.batch})")
    print(f"모델: {args.model}")
    print(f"신뢰도 임계값: {args.conf}\n")

    if not batch:
        print("처리할 이미지가 없습니다. 모든 이미지가 라벨링되었습니다.")
        return

    # 모델 로드
    if not os.path.exists(args.model):
        print(f"[오류] 모델 파일을 찾을 수 없습니다: {args.model}")
        print("먼저 train_seg.py를 실행하여 모델을 학습하세요.")
        return

    model = YOLO(args.model)
    print("모델 로드 완료\n")

    labeled = 0
    skipped = 0   # 접시 미감지

    for i, img_file in enumerate(batch):
        img_path = os.path.join(IMAGE_DIR, img_file)
        results = model(img_path, verbose=False, conf=args.conf)

        label_lines = []
        for r in results:
            if r.masks is None:
                continue
            for mask, box in zip(r.masks.xyn, r.boxes):
                cls = int(box.cls[0])
                points = mask.flatten().tolist()
                line = f"{cls} " + " ".join(f"{p:.6f}" for p in points)
                label_lines.append(line)

        if label_lines:
            label_path = os.path.join(LABEL_DIR, img_file.replace('.jpg', '.txt'))
            with open(label_path, 'w') as f:
                f.write('\n'.join(label_lines))
            labeled += 1
        else:
            skipped += 1

        # 진행 상황
        done = i + 1
        if done % 50 == 0 or done == len(batch):
            print(f"  [{done}/{len(batch)}] 라벨 생성: {labeled} / 미감지 스킵: {skipped}")

    print(f"\n완료!")
    print(f"  라벨 생성: {labeled}장")
    print(f"  미감지 스킵: {skipped}장")
    print(f"  labels_v2 총 라벨 수: {len(os.listdir(LABEL_DIR))}장")
    print(f"  남은 미라벨 이미지: {total_remaining - len(batch)}장")
    print(f"\n다음 단계:")
    print(f"  1. labels_v2 의 새 라벨을 CVAT/Roboflow에서 검수·수정")
    print(f"  2. train_seg.py 재실행 → 재학습")
    print(f"  3. auto_label_seg.py 재실행 → 다음 배치 처리")

if __name__ == '__main__':
    main()
