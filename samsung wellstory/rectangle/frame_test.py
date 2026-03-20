import cv2
import os

# 1. 파일 경로 설정
video_path = r'C:\Users\ASUS\Desktop\bowl_video.mp4'  # 영상 파일 이름
output_folder = r'C:\Users\ASUS\Desktop\datasets\images' # 저장될 폴더명

# 2. 폴더가 없으면 생성
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

# 3. 영상 불러오기
cap = cv2.VideoCapture(video_path)
start_index = 1500  # 기존 1500장 이후부터 시작
count = 0
total = 500   # 추가로 추출할 장수
interval = 3  # 매 3프레임마다 1장 추출

print("프레임 추출을 시작합니다...")

frame_num = 0
while cap.isOpened():
    ret, frame = cap.read()
    if not ret or count >= total:
        break

    if frame_num % interval == 0:
        file_name = os.path.join(output_folder, f"bowl_{start_index + count:05d}.jpg")
        cv2.imwrite(file_name, frame)
        count += 1
        if count % 100 == 0:
            print(f"{count}장 저장 완료...")

    frame_num += 1

cap.release()
print("작업 완료!")