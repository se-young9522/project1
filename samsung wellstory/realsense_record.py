"""
RealSense 영상 녹화 스크립트
- mp4 파일로 저장
- 스페이스바: 녹화 시작/정지
- q: 종료
"""
import pyrealsense2 as rs
import numpy as np
import cv2
import os
from datetime import datetime

# ─── 설정 ─────────────────────────────────────────
output_folder = r'C:\Users\ASUS\Desktop\datasets\videos'
RESOLUTION = (1280, 720)
FPS = 30
# ────────────────────────────────────────────────────

os.makedirs(output_folder, exist_ok=True)

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, RESOLUTION[0], RESOLUTION[1], rs.format.bgr8, FPS)
pipeline.start(config)

recording = False
writer = None
video_count = 0
frame_count = 0

print("RealSense 영상 녹화")
print("─" * 40)
print("스페이스바 : 녹화 시작/정지")
print("Q          : 종료")
print("─" * 40)

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        img = np.asanyarray(color_frame.get_data())

        # 화면 표시
        display = img.copy()
        if recording:
            # 녹화 중 빨간 원 표시
            cv2.circle(display, (30, 30), 12, (0, 0, 255), -1)
            cv2.putText(display, f"REC  {frame_count} frames ({frame_count/FPS:.1f}s)",
                        (50, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            writer.write(img)
            frame_count += 1
        else:
            cv2.putText(display, "SPACE: start recording  Q: quit",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow('RealSense Record', display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):
            if not recording:
                # 녹화 시작
                video_count += 1
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                video_path = os.path.join(output_folder, f"bowl_{timestamp}.mp4")
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                writer = cv2.VideoWriter(video_path, fourcc, FPS, RESOLUTION)
                recording = True
                frame_count = 0
                print(f"녹화 시작: {video_path}")
            else:
                # 녹화 정지
                recording = False
                writer.release()
                writer = None
                print(f"녹화 정지: {frame_count}프레임 ({frame_count/FPS:.1f}초)")
        elif key == ord('q'):
            break

finally:
    if writer:
        writer.release()
    pipeline.stop()
    cv2.destroyAllWindows()
    print(f"\n완료! 총 {video_count}개 영상 저장됨 → {output_folder}")
