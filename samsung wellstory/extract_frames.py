"""
mp4 영상에서 프레임 추출
- videos 폴더의 모든 mp4 파일에서 프레임 추출
- 중복 방지: 일정 간격으로 추출 (기본 3프레임마다 1장)
"""
import cv2
import os

# ─── 설정 ─────────────────────────────────────────
video_folder = r'C:\Users\ASUS\Desktop\datasets\videos'
output_folder = r'C:\Users\ASUS\Desktop\datasets\videos\images'
frame_interval = 3   # 3프레임마다 1장 추출 (30fps → 10fps)
# ────────────────────────────────────────────────────

os.makedirs(output_folder, exist_ok=True)

# 기존 이미지 번호 이어서
existing = [f for f in os.listdir(output_folder) if f.startswith('bowl_') and f.endswith('.jpg')]
start_index = max(
    [int(f.replace('bowl_', '').replace('.jpg', '')) for f in existing],
    default=-1
) + 1
print(f"기존 이미지 {len(existing)}장, bowl_{start_index:05d}.jpg부터 저장")

videos = sorted(f for f in os.listdir(video_folder) if f.endswith('.mp4'))
print(f"영상 파일: {len(videos)}개")

total_saved = 0
idx = start_index

for vname in videos:
    vpath = os.path.join(video_folder, vname)
    cap = cv2.VideoCapture(vpath)
    fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"\n{vname}: {total_frames}프레임 ({total_frames/fps:.1f}초)")

    frame_num = 0
    saved = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if frame_num % frame_interval == 0:
            fname = os.path.join(output_folder, f"bowl_{idx:05d}.jpg")
            cv2.imwrite(fname, frame)
            idx += 1
            saved += 1

        frame_num += 1

    cap.release()
    total_saved += saved
    print(f"  추출: {saved}장")

print(f"\n완료! 총 {total_saved}장 추출 (bowl_{start_index:05d} ~ bowl_{idx-1:05d})")
print(f"frame_interval={frame_interval} (30fps에서 {30//frame_interval}fps로 추출)")
