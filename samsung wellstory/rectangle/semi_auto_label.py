from ultralytics import YOLO
import os

# ─── 경로 설정 ─────────────────────────────────────────
image_folder  = r'C:\Users\ASUS\Desktop\datasets\images'
label_folder  = r'C:\Users\ASUS\Desktop\datasets\labels'
model_path    = r'C:\Users\ASUS\runs\detect\bowl_v14\weights\best.pt'
# ────────────────────────────────────────────────────────

model = YOLO(model_path)
print(f"학습된 모델 로드 완료: {model_path}\n")

# 아직 라벨이 없는 이미지만 대상
all_images = [f for f in os.listdir(image_folder) if f.endswith('.jpg')]
unlabeled  = [
    f for f in all_images
    if not os.path.exists(os.path.join(label_folder, f.replace('.jpg', '.txt')))
]
print(f"자동 라벨링 대상: {len(unlabeled)}장 (이미 라벨된 이미지는 건너뜀)\n")

labeled = 0
skipped = 0

for img_file in unlabeled:
    img_path = os.path.join(image_folder, img_file)
    results  = model(img_path, verbose=False, conf=0.35, device='cpu')

    label_lines = []
    for r in results:
        for box in r.boxes:
            cx, cy, w, h = box.xywhn[0].tolist()
            conf = float(box.conf[0])
            label_lines.append(f"0 {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")  # 클래스 0 = bowl

    if label_lines:
        label_file = os.path.join(label_folder, img_file.replace('.jpg', '.txt'))
        with open(label_file, 'w') as f:
            f.write('\n'.join(label_lines))
        labeled += 1
    else:
        skipped += 1

    if (labeled + skipped) % 100 == 0:
        print(f"진행: {labeled + skipped}/{len(unlabeled)} | 라벨: {labeled} / 스킵: {skipped}")

print(f"\n완료! 자동 라벨 생성: {labeled}장 / 스킵: {skipped}장")
print("이후 전체 데이터로 2차 학습을 진행하세요 (train_v1.py 재실행)")
