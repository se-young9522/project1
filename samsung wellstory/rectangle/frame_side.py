import cv2
import os

# 1. 파일 경로 설정
video_path    = r'C:\Users\ASUS\Desktop\bowl_video.mp4'
output_folder = r'C:\Users\ASUS\Desktop\datasets\images'

if not os.path.exists(output_folder):
    os.makedirs(output_folder)

# 2. 측면 구간 설정 (초 단위)
side_sections = [
    (45, 58),       # 45초 ~ 58초
    (69, 75),       # 1분9초 ~ 1분15초
    (88, 99),       # 1분28초 ~ 1분39초
]

# 3. 기존 이미지 중 가장 큰 번호 찾기 (이어서 저장)
existing = [f for f in os.listdir(output_folder) if f.startswith('bowl_') and f.endswith('.jpg')]
start_index = max([int(f.replace('bowl_','').replace('.jpg','')) for f in existing], default=-1) + 1
print(f"기존 이미지 {len(existing)}장 확인 → bowl_{start_index:05d}.jpg 부터 저장\n")

# 4. 영상 정보
cap = cv2.VideoCapture(video_path)
fps = cap.get(cv2.CAP_PROP_FPS)
print(f"영상 FPS: {fps}")

count = 0
saved = 0
interval = 3  # 매 3프레임마다 1장
max_save = 300  # 최대 300장

for (start_sec, end_sec) in side_sections:
    if saved >= max_save:
        break
    start_frame = int(start_sec * fps)
    end_frame   = int(end_sec * fps)
    cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)

    print(f"\n구간 {start_sec}초 ~ {end_sec}초 추출 중...")
    for i in range(end_frame - start_frame):
        if saved >= max_save:
            break
        ret, frame = cap.read()
        if not ret:
            break
        if i % interval == 0:
            file_name = os.path.join(output_folder, f"bowl_{start_index + saved:05d}.jpg")
            cv2.imwrite(file_name, frame)
            saved += 1

    print(f"  → 저장: {saved}장")

cap.release()
print(f"\n전체 완료! 측면 이미지 {saved}장 추가 저장 (bowl_{start_index:05d} ~ bowl_{start_index+saved-1:05d})")
