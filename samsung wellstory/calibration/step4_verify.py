"""
4단계: 캘리브레이션 검증 + 통합 테스트
- YOLO로 접시 인식 → 카메라 3D 좌표 → 변환 행렬로 로봇 좌표 변환
- 변환된 좌표를 화면에 표시 (로봇 이동 전 확인)
- 확인 후 로봇을 접시 위 50mm로 이동 (안전 높이)

사용법: python step4_verify.py
  마우스 클릭: 해당 위치의 변환 좌표 확인 (수동)
  d: YOLO로 접시 자동 인식 → 변환 좌표 표시
  g: 인식된 접시 위 50mm로 로봇 이동 (Go!)
  q: 종료
"""
import sys
import numpy as np
import cv2
import pyrealsense2 as rs
import os
# ─── 경로 설정 ───
CALIB_DIR = r'C:\Users\ASUS\Desktop\samsung wellstory\calibration'
MODEL_PATH = r'C:\Users\ASUS\Desktop\samsung wellstory\runs\segment\bowl_seg_v53\bowl_seg_v53\weights\best.pt'
SAFE_HEIGHT = 200  # 접시 위 200mm에서 먼저 정지
CAMERA_TILT_DEG = 55.0  # 수직 기준 카메라 기울기 (depth 측정값 기준)

# ─── 로봇 좌표 오프셋 보정 (mm) ───
# 로봇이 실제 접시 중심보다 아래+우측으로 2cm씩 밀림 → 반대 방향으로 보정
# 부호가 반대면 ±를 뒤집어서 조정
OFFSET_X_MM = -20  # 좌측 보정
OFFSET_Y_MM = 25   # Y+ 방향 2.5cm 보정

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

# ─── YOLO 모델 로드 ───
from ultralytics import YOLO
model = YOLO(MODEL_PATH)
print(f"[OK] YOLO 모델 로드: bowl_seg_v53")

# ─── 로봇 연결 ───
sys.path.insert(0, r'C:\Users\ASUS\Desktop\samsung wellstory\RM_API-main\Example\API_Example_Python\API_Example_Python')
from robotic_arm_package.robotic_arm import *

robot = Arm(RM65, "192.168.1.18")
print(f"[OK] 로봇 연결 (API: {robot.API_Version()})")

# 아래를 향하는 자세 (고정값 - 엔드툴이 아래를 향할 때의 값)
# ※ 로봇을 아래 향하는 자세로 놓고 check_tcp_now.py로 확인한 값
DOWN_RX = 3.103
DOWN_RY = 0.051
DOWN_RZ = 0.661
print(f"     고정 자세: rx={DOWN_RX:.3f} ry={DOWN_RY:.3f} rz={DOWN_RZ:.3f}")

MIN_Z = 0.080  # 최소 Z 높이 100mm (이 이하로는 이동 안 함)
print(f"     최소 Z 높이: {MIN_Z*1000:.0f}mm")

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
detected_bowl = None  # (cam_xyz_mm, robot_xyz_mm, pixel_cx, pixel_cy)

def on_mouse(event, x, y, flags, param):
    global mouse_x, mouse_y, click_pos
    mouse_x, mouse_y = x, y
    if event == cv2.EVENT_LBUTTONDOWN:
        click_pos = (x, y)

cv2.namedWindow('Step4 - Verify')
cv2.setMouseCallback('Step4 - Verify', on_mouse)

print()
print("=" * 60)
print("  4단계: 캘리브레이션 검증 + 통합 테스트")
print("=" * 60)
print("  d: YOLO 접시 인식 → 변환 좌표 표시")
print("  g: 인식된 접시 위 50mm로 로봇 이동")
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

        # ─── 인식된 접시 표시 ───
        if detected_bowl:
            cam_xyz, rob_xyz, bcx, bcy = detected_bowl
            cv2.circle(display, (bcx, bcy), 15, (0, 0, 255), 3)
            cv2.putText(display, f"Bowl detected!", (bcx + 20, bcy - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(display, f"Robot: ({rob_xyz[0]:.0f}, {rob_xyz[1]:.0f}, {rob_xyz[2]:.0f})mm",
                        (bcx + 20, bcy + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(display, f"Press 'g' to move robot (safe height +{SAFE_HEIGHT}mm)",
                        (bcx + 20, bcy + 35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # 안내
        cv2.putText(display, "d: detect | g: go to bowl | click: manual check | q: quit",
                    (10, 700), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        cv2.imshow('Step4 - Verify', display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break

        elif key == ord('d'):
            # YOLO 접시 인식 (seg 마스크 무게중심)
            results = model(color_image, conf=0.7, verbose=False)
            best_conf = 0
            best_bowl = None

            for r in results:
                if r.masks is not None:
                    for i, box in enumerate(r.boxes):
                        conf = float(box.conf[0])
                        if conf > best_conf:
                            # seg 마스크 윤곽 → 3D 변환 → 3D 중심 계산
                            mask = r.masks.data[i].cpu().numpy()
                            mask_resized = cv2.resize(mask, (1280, 720))
                            mask_uint8 = (mask_resized > 0.5).astype(np.uint8) * 255
                            contours_bowl, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                            if contours_bowl:
                                largest = max(contours_bowl, key=cv2.contourArea)
                                if len(largest) >= 5:
                                    # 1) 2D ellipse fitting → 정확한 픽셀 중심
                                    ell = cv2.fitEllipse(largest)
                                    ecx, ecy = int(ell[0][0]), int(ell[0][1])

                                    # 2) 테두리(rim) depth 사용 (중심 depth는 접시 안쪽이라 부정확)
                                    rim_depths = []
                                    contour_pts = largest.reshape(-1, 2)
                                    # 윤곽점 중 균등하게 샘플링
                                    step = max(1, len(contour_pts) // 40)
                                    for pt in contour_pts[::step]:
                                        px2 = max(0, min(1279, pt[0]))
                                        py2 = max(0, min(719, pt[1]))
                                        dd = depth_filtered.get_distance(px2, py2)
                                        if dd > 0.1:
                                            rim_depths.append(dd)

                                    if len(rim_depths) >= 5:
                                        # 림 depth 중 가장 가까운 쪽 (카메라에 가까운 = 윗면)
                                        rim_depth = np.percentile(rim_depths, 25)

                                        # 3) ellipse 중심 + rim depth로 3D deproject
                                        p3 = rs.rs2_deproject_pixel_to_point(intrinsics, [ecx, ecy], rim_depth)
                                        center_3d = np.array([p3[0]*1000, p3[1]*1000, p3[2]*1000])

                                        # 4) 카메라 기울기 보정 (55도에서 타원 중심 편향 보정)
                                        # 타원 장축/단축 비율로 실제 viewing angle 확인
                                        major_ax = max(ell[1])
                                        minor_ax = min(ell[1])
                                        if major_ax > 0:
                                            # 타원 장축 방향 = 카메라 기울기 방향
                                            ell_angle_rad = np.deg2rad(ell[2])
                                            # 접시 반경 추정 (장축 = 실제 직경의 투영)
                                            bowl_radius_px = major_ax / 2
                                            # 편향 보정: 접시 높이(깊이)에 의한 중심 이동
                                            # 접시 깊이 ~30mm 가정, sin(55°) ≈ 0.82
                                            bowl_depth_mm = 30  # 접시 깊이 (조절 가능)
                                            tilt_rad = np.deg2rad(CAMERA_TILT_DEG)
                                            # 3D 공간에서 카메라 반대 방향으로 보정
                                            shift_mm = bowl_depth_mm * np.sin(tilt_rad) * 0.5
                                            # 카메라 Y축 방향 (아래쪽)으로 편향 → 반대로 보정
                                            center_3d[1] -= shift_mm * np.cos(tilt_rad)
                                            center_3d[2] -= shift_mm * np.sin(tilt_rad)

                                        best_conf = conf
                                        best_bowl = (center_3d, conf, largest, ecx, ecy)

            if best_bowl:
                center_3d, conf, largest_contour, bcx, bcy = best_bowl
                cam_xyz = center_3d.tolist()
                rob_xyz = cam_to_robot(cam_xyz)
                detected_bowl = (cam_xyz, rob_xyz, bcx, bcy)
                print(f"  [접시 인식] conf={conf:.0%} pixel=({bcx},{bcy}) (ellipse center + rim depth + {CAMERA_TILT_DEG:.0f}° 보정)")
                print(f"    카메라: ({cam_xyz[0]:.1f}, {cam_xyz[1]:.1f}, {cam_xyz[2]:.1f})mm")
                print(f"    로봇:   ({rob_xyz[0]:.1f}, {rob_xyz[1]:.1f}, {rob_xyz[2]:.1f})mm")
                print(f"    → 'g'키로 접시 위 {SAFE_HEIGHT}mm로 이동")
            else:
                print("  [!] 접시를 인식하지 못했습니다")
                detected_bowl = None

        elif key == ord('g') and detected_bowl:
            # 접시 위 안전 높이로 이동
            cam_xyz, rob_xyz, bcx, bcy = detected_bowl

            # 로봇 좌표 (mm → m 변환)
            target_x = rob_xyz[0] / 1000  # mm → m
            target_y = rob_xyz[1] / 1000
            target_z = rob_xyz[2] / 1000 + SAFE_HEIGHT / 1000  # +50mm 안전 높이

            # Z 최소 높이 체크
            if target_z < MIN_Z:
                print(f"\n  [!] Z={target_z*1000:.1f}mm → 너무 낮음! (최소 {MIN_Z*1000:.0f}mm)")
                print(f"      캘리브레이션이 잘못되었을 수 있습니다")
                continue

            target_pose = [target_x, target_y, target_z, DOWN_RX, DOWN_RY, DOWN_RZ]

            print(f"\n  [이동] 접시 위 {SAFE_HEIGHT}mm로 이동합니다")
            print(f"    목표: ({target_x*1000:.1f}, {target_y*1000:.1f}, {target_z*1000:.1f})mm")
            print(f"    자세: rx={DOWN_RX:.3f} ry={DOWN_RY:.3f} rz={DOWN_RZ:.3f}")
            print(f"    속도: v=20 (저속)")
            print(f"    !! 비상정지 버튼 준비 !!")

            result = robot.Movej_P_Cmd(target_pose, v=80, r=0)
            if result and '1' in str(result):
                print(f"  [!] 이동 실패! 로봇이 해당 위치에 도달할 수 없습니다")
                print(f"      → 목표가 작업 범위 밖이거나 자세가 불가능")
            else:
                print(f"  [완료] 이동 명령 전송됨 (Movej_P: 관절 공간 이동)")
                print(f"    → 로봇이 접시 바로 위에 있는지 눈으로 확인하세요")
                print(f"    → 위치가 맞으면 캘리브레이션 성공!")

finally:
    pipeline.stop()
    robot.Arm_Socket_Close()
    cv2.destroyAllWindows()
    print("\n[OK] 종료")
