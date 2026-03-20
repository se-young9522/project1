from ultralytics import YOLO
import os

# ─── 경로 설정 ───────────────────────────────────────────────
image_folder = r'C:\Users\ASUS\Desktop\datasets\images'
label_folder = r'C:\Users\ASUS\Desktop\datasets\labels'
# ────────────────────────────────────────────────────────────

os.makedirs(label_folder, exist_ok=True)

# COCO 사전학습 모델 로드 (처음 실행 시 자동 다운로드)
model = YOLO('yolov8x.pt')

BOWL_CLASS_COCO = 45  # COCO에서 bowl 클래스 번호

image_files = [f for f in os.listdir(image_folder) if f.endswith('.jpg')]
total = len(image_files)
print(f"총 {total}장 라벨링 시작...")

labeled = 0
skipped = 0

for i, img_file in enumerate(image_files):
    img_path = os.path.join(image_folder, img_file)
    results = model(img_path, verbose=False)

    label_lines = []
    for r in results:
        for box in r.boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])

            # bowl 클래스이고 신뢰도 50% 이상인 것만
            if cls == BOWL_CLASS_COCO and conf >= 0.5:
                cx, cy, w, h = box.xywhn[0].tolist()  # 0~1 정규화 좌표
                label_lines.append(f"0 {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")

    if label_lines:
        label_file = os.path.join(label_folder, img_file.replace('.jpg', '.txt'))
        with open(label_file, 'w') as f:
            f.write('\n'.join(label_lines))
        labeled += 1
    else:
        skipped += 1  # 그릇이 감지되지 않은 프레임

    # 진행 상황 출력
    if (i + 1) % 200 == 0 or (i + 1) == total:
        print(f"[{i+1}/{total}] 라벨 생성: {labeled}장 / 스킵: {skipped}장")

print(f"\n✔ 완료! 라벨 생성: {labeled}장 / 스킵(그릇 없음): {skipped}장")
print(f"라벨 저장 위치: {label_folder}")
