"""
다중 접시 순차 Pick & Place 테스트
- YOLO로 모든 접시 인식 → Start Point 기준 가까운 순 정렬
- Start → Bowl1 → Start → Bowl2 → Start ... 순차 이동

사용법: python test.py
  s: 현재 로봇 관절각을 Start Point로 저장
  d: YOLO로 모든 접시 인식 → 거리 순 정렬 표시
  g: 모든 접시 순차 이동 (Go!)
  마우스 클릭: 해당 위치의 변환 좌표 확인 (수동)
  q: 종료
"""
import sys
import numpy as np
import cv2
import pyrealsense2 as rs
import os
import math

# ─── 경로 설정 ───
CALIB_DIR = r'C:\Users\ASUS\Desktop\samsung wellstory\calibration'
MODEL_PATH = r'C:\Users\ASUS\Desktop\samsung wellstory\runs\segment\bowl_seg_v53\bowl_seg_v53\weights\best.pt'
SAFE_HEIGHT = 200  # 접시 위 200mm에서 먼저 정지
CAMERA_TILT_DEG = 55.0  # 수직 기준 카메라 기울기

# ─── 로봇 좌표 오프셋 보정 (mm) ───
OFFSET_X_MM = -20
OFFSET_Y_MM = 25

# ─── Start Point (관절각 + TCP) ───
START_JOINTS = None  # 's'키로 저장
START_TCP_XY = None  # Start Point의 로봇 XY 좌표 (mm) - 거리 계산용

# ─── 속도 설정 ───
V_MOVE = 100 # mm/s (Movej_P 속도)

# ─── 캘리브레이션 행렬 로드 ───
matrix_path = os.path.join(CALIB_DIR, 'calibration_matrix.npy')
if not os.path.exists(matrix_path):
    print("[FAIL] calibration_matrix.npy 없음! 3단계를 먼저 실행하세요")
    sys.exit(1)

T = np.load(matrix_path)
R = T[:3, :3]
t = T[:3, 3]
print(f"[OK] 캘리브레이션 행렬 로드: {matrix_path}")

def cam_to_robot(cam_xyz_mm):
    """카메라 좌표(mm) → 로봇 좌표(mm) 변환 + 오프셋 보정"""
    cam = np.array(cam_xyz_mm)
    rob = R @ cam + t
    rob[0] += OFFSET_X_MM
    rob[1] += OFFSET_Y_MM
    return rob.tolist()

def dist_from_start(rob_xyz):
    """로봇 XY 좌표 기준 Start Point까지 거리 (mm)"""
    if START_TCP_XY is None:
        return 0
    dx = rob_xyz[0] - START_TCP_XY[0]
    dy = rob_xyz[1] - START_TCP_XY[1]
    return math.sqrt(dx*dx + dy*dy)

def detect_bowl_center(mask_data, depth_filtered, intrinsics):
    """seg 마스크 → ellipse fitting + rim depth + 기울기 보정 → 3D 중심 반환"""
    mask_resized = cv2.resize(mask_data, (1280, 720))
    mask_uint8 = (mask_resized > 0.5).astype(np.uint8) * 255
    contours_bowl, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours_bowl:
        return None, None, None

    largest = max(contours_bowl, key=cv2.contourArea)
    if len(largest) < 5:
        return None, None, None

    # 1) ellipse fitting → 픽셀 중심
    ell = cv2.fitEllipse(largest)
    ecx, ecy = int(ell[0][0]), int(ell[0][1])

    # 2) rim depth (윤곽점 샘플링)
    rim_depths = []
    contour_pts = largest.reshape(-1, 2)
    step = max(1, len(contour_pts) // 40)
    for pt in contour_pts[::step]:
        px2 = max(0, min(1279, pt[0]))
        py2 = max(0, min(719, pt[1]))
        dd = depth_filtered.get_distance(px2, py2)
        if dd > 0.1:
            rim_depths.append(dd)

    if len(rim_depths) < 5:
        return None, None, None

    rim_depth = np.percentile(rim_depths, 25)

    # 3) 3D deproject
    p3 = rs.rs2_deproject_pixel_to_point(intrinsics, [ecx, ecy], rim_depth)
    center_3d = np.array([p3[0]*1000, p3[1]*1000, p3[2]*1000])

    # 4) 카메라 기울기 보정
    major_ax = max(ell[1])
    if major_ax > 0:
        bowl_depth_mm = 30
        tilt_rad = np.deg2rad(CAMERA_TILT_DEG)
        shift_mm = bowl_depth_mm * np.sin(tilt_rad) * 0.5
        center_3d[1] -= shift_mm * np.cos(tilt_rad)
        center_3d[2] -= shift_mm * np.sin(tilt_rad)

    return center_3d, ecx, ecy

# ─── YOLO 모델 로드 ───
from ultralytics import YOLO
model = YOLO(MODEL_PATH)
print(f"[OK] YOLO 모델 로드: bowl_seg_v53")

# ─── 로봇 연결 ───
sys.path.insert(0, r'C:\Users\ASUS\Desktop\samsung wellstory\RM_API-main\Example\API_Example_Python\API_Example_Python')
from robotic_arm_package.robotic_arm import *

robot = Arm(RM65, "192.168.1.18")
print(f"[OK] 로봇 연결 (API: {robot.API_Version()})")

DOWN_RX = 3.103
DOWN_RY = 0.051
DOWN_RZ = 0.661
print(f"     고정 자세: rx={DOWN_RX:.3f} ry={DOWN_RY:.3f} rz={DOWN_RZ:.3f}")

MIN_Z = 0.080
print(f"     최소 Z 높이: {MIN_Z*1000:.0f}mm")
print(f"     속도: v={V_MOVE}")
print(f"     Start Point: 미설정 ('s'키로 저장하세요)")

# ─── RealSense 연결 ───
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
pipeline.start(config)
align = rs.align(rs.stream.color)

spatial = rs.spatial_filter()
temporal = rs.temporal_filter()

for _ in range(30):
    pipeline.wait_for_frames()
print("[OK] RealSense 준비 완료")

# ─── 상태 변수 ───
mouse_x, mouse_y = 640, 360
click_pos = None
detected_bowls = []  # [(cam_xyz, rob_xyz, bcx, bcy), ...] 거리 순 정렬

def on_mouse(event, x, y, flags, param):
    global mouse_x, mouse_y, click_pos
    mouse_x, mouse_y = x, y
    if event == cv2.EVENT_LBUTTONDOWN:
        click_pos = (x, y)

cv2.namedWindow('Multi-Bowl Test')
cv2.setMouseCallback('Multi-Bowl Test', on_mouse)

print()
print("=" * 60)
print("  다중 접시 순차 Pick & Place 테스트")
print("=" * 60)
print("  s: 현재 관절각을 Start Point로 저장")
print("  d: YOLO 모든 접시 인식 → 거리 순 정렬")
print("  g: 모든 접시 순차 이동 (Start→Bowl1→Start→Bowl2→Start...)")
print("  마우스 클릭: 해당 위치의 변환 좌표 수동 확인")
print("  q: 종료")
print("=" * 60)

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        filtered = spatial.process(depth_frame)
        filtered = temporal.process(filtered)
        depth_filtered = filtered.as_depth_frame()

        color_image = np.asanyarray(color_frame.get_data())
        intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
        display = color_image.copy()

        # ─── 마우스 위치 실시간 변환 표시 ───
        depth_m = depth_filtered.get_distance(mouse_x, mouse_y)
        point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [mouse_x, mouse_y], depth_m)
        cam_mm = [p * 1000 for p in point_3d]
        rob_mm = cam_to_robot(cam_mm)

        cv2.drawMarker(display, (mouse_x, mouse_y), (0, 255, 0), cv2.MARKER_CROSS, 20, 1)
        cv2.putText(display, f"Camera: ({cam_mm[0]:.0f}, {cam_mm[1]:.0f}, {cam_mm[2]:.0f})mm",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        cv2.putText(display, f"Robot:  ({rob_mm[0]:.0f}, {rob_mm[1]:.0f}, {rob_mm[2]:.0f})mm",
                    (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

        # ─── 클릭 시 변환 좌표 출력 ───
        if click_pos:
            cx, cy = click_pos
            d = depth_filtered.get_distance(cx, cy)
            if d > 0.1:
                p = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], d)
                c_mm = [p[0]*1000, p[1]*1000, p[2]*1000]
                r_mm = cam_to_robot(c_mm)
                print(f"  [클릭] pixel=({cx},{cy})")
                print(f"         카메라: ({c_mm[0]:.1f}, {c_mm[1]:.1f}, {c_mm[2]:.1f})mm")
                print(f"         로봇:   ({r_mm[0]:.1f}, {r_mm[1]:.1f}, {r_mm[2]:.1f})mm")
            click_pos = None

        # ─── 인식된 접시들 표시 ───
        for idx, (cam_xyz, rob_xyz, bcx, bcy) in enumerate(detected_bowls):
            color = (0, 0, 255) if idx == 0 else (0, 165, 255)  # 1번=빨강, 나머지=주황
            cv2.circle(display, (bcx, bcy), 15, color, 3)
            cv2.putText(display, f"#{idx+1}", (bcx - 10, bcy - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            cv2.putText(display, f"({rob_xyz[0]:.0f}, {rob_xyz[1]:.0f}, {rob_xyz[2]:.0f})mm",
                        (bcx + 20, bcy + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            dist = dist_from_start(rob_xyz)
            cv2.putText(display, f"dist={dist:.0f}mm",
                        (bcx + 20, bcy + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        # 안내
        start_status = "SET" if START_JOINTS else "NOT SET (press 's')"
        bowl_status = f"{len(detected_bowls)}bowls" if detected_bowls else "none"
        cv2.putText(display, f"Start: {start_status} | Bowls: {bowl_status} | s/d/g/q",
                    (10, 700), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        cv2.imshow('Multi-Bowl Test', display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break

        elif key == ord('s'):
            ret = robot.Get_Current_Arm_State()
            if ret and len(ret) >= 3 and ret[1]:
                START_JOINTS = list(ret[1])
                tcp = ret[2]
                # TCP XY 저장 (거리 계산용, mm 단위)
                START_TCP_XY = (tcp[0]*1000, tcp[1]*1000)
                print(f"\n  [Start Point 저장]")
                print(f"    관절각: {[f'{j:.1f}' for j in START_JOINTS]}")
                print(f"    TCP: ({tcp[0]*1000:.1f}, {tcp[1]*1000:.1f}, {tcp[2]*1000:.1f})mm")
            else:
                print(f"\n  [!] 상태 읽기 실패: {ret}")

        elif key == ord('d'):
            # 모든 접시 인식
            results = model(color_image, conf=0.7, verbose=False)
            bowls_found = []

            for r in results:
                if r.masks is not None:
                    for i, box in enumerate(r.boxes):
                        conf = float(box.conf[0])
                        mask = r.masks.data[i].cpu().numpy()
                        center_3d, ecx, ecy = detect_bowl_center(mask, depth_filtered, intrinsics)
                        if center_3d is not None:
                            cam_xyz = center_3d.tolist()
                            rob_xyz = cam_to_robot(cam_xyz)
                            bowls_found.append((cam_xyz, rob_xyz, ecx, ecy, conf))

            if bowls_found:
                # Start Point 기준 거리 순 정렬 (가까운 순)
                bowls_found.sort(key=lambda b: dist_from_start(b[1]))
                detected_bowls = [(b[0], b[1], b[2], b[3]) for b in bowls_found]
                print(f"\n  [접시 인식] {len(detected_bowls)}개 발견 (거리 순 정렬)")
                for idx, (cam_xyz, rob_xyz, bcx, bcy) in enumerate(detected_bowls):
                    d = dist_from_start(rob_xyz)
                    print(f"    #{idx+1} pixel=({bcx},{bcy}) 로봇=({rob_xyz[0]:.0f},{rob_xyz[1]:.0f},{rob_xyz[2]:.0f})mm dist={d:.0f}mm")
                print(f"  → 'g'키로 순차 이동 시작")
            else:
                print("  [!] 접시를 인식하지 못했습니다")
                detected_bowls = []

        elif key == ord('g'):
            if not START_JOINTS:
                print(f"\n  [!] Start Point 미설정! 's'키로 먼저 저장하세요")
                continue
            if not detected_bowls:
                print(f"\n  [!] 인식된 접시 없음! 'd'키로 먼저 인식하세요")
                continue

            print(f"\n  ===== 다중 접시 순차 이동 시작 ({len(detected_bowls)}개) =====")
            print(f"  !! 비상정지 버튼 준비 !!")

            for idx, (cam_xyz, rob_xyz, bcx, bcy) in enumerate(detected_bowls):
                target_x = rob_xyz[0] / 1000
                target_y = rob_xyz[1] / 1000
                target_z = rob_xyz[2] / 1000 + SAFE_HEIGHT / 1000

                if target_z < MIN_Z:
                    print(f"\n  [#{idx+1}] Z={target_z*1000:.1f}mm → 너무 낮음! 스킵")
                    continue

                target_pose = [target_x, target_y, target_z, DOWN_RX, DOWN_RY, DOWN_RZ]

                print(f"\n  ----- Bowl #{idx+1}/{len(detected_bowls)} -----")

                # Step A: Start Point로 이동
                print(f"  [A] Start Point로 이동")
                result = robot.Movej_Cmd(START_JOINTS, v=V_MOVE, r=0)
                if result and '1' in str(result):
                    print(f"  [!] Start Point 이동 실패! 중단")
                    break

                # Step B: 접시 위로 이동
                print(f"  [B] Bowl #{idx+1} 위로 이동 ({target_x*1000:.0f}, {target_y*1000:.0f}, {target_z*1000:.0f})mm")
                result = robot.Movej_P_Cmd(target_pose, v=V_MOVE, r=0)
                if result and '1' in str(result):
                    print(f"  [!] Bowl #{idx+1} 이동 실패! Start Point 복귀 후 중단")
                    robot.Movej_Cmd(START_JOINTS, v=V_MOVE, r=0)
                    break

            # 마지막: Start Point로 최종 복귀
            print(f"\n  [최종] Start Point로 복귀")
            robot.Movej_Cmd(START_JOINTS, v=V_MOVE, r=0)
            print(f"  ===== 완료 =====")

finally:
    pipeline.stop()
    robot.Arm_Socket_Close()
    cv2.destroyAllWindows()
    print("\n[OK] 종료")
