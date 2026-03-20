"""
3단계: Eye-to-Hand 캘리브레이션
- 고정 카메라 ↔ 로봇 베이스 간의 변환 행렬(T) 계산
- 로봇 TCP(플랜지 볼트 중심)를 여러 위치로 이동 → 카메라에서 클릭 → 좌표 쌍 수집
- 최소 5개, 권장 8~10개 포인트 수집

사용법: python step3_calibration.py
  1. 로봇을 원하는 위치로 이동 (수동 또는 webapp)
  2. 카메라 화면에서 플랜지 볼트 중심을 마우스 클릭
  3. Space키: 해당 포인트 저장 (카메라 3D + 로봇 TCP 동시 기록)
  4. 5개 이상 모이면 c키로 캘리브레이션 계산
  s: 중간 저장  |  c: 캘리브레이션 계산  |  z: 마지막 포인트 삭제  |  q: 종료
"""
import sys
import numpy as np
import cv2
import pyrealsense2 as rs
import json
import os
from datetime import datetime

# ─── 로봇 연결 ───
sys.path.insert(0, r'C:\Users\ASUS\Desktop\samsung wellstory\RM_API-main\Example\API_Example_Python\API_Example_Python')
from robotic_arm_package.robotic_arm import *

SAVE_DIR = r'C:\Users\ASUS\Desktop\samsung wellstory\calibration'
os.makedirs(SAVE_DIR, exist_ok=True)

print("=" * 60)
print("  3단계: Eye-to-Hand 캘리브레이션")
print("=" * 60)

# ─── 로봇 연결 ───
print("\n[로봇 연결 중...]")
robot = Arm(RM65, "192.168.1.18")
print(f"[OK] 로봇 연결 (API: {robot.API_Version()})")

ret = robot.Get_Current_Arm_State(retry=1)
print(f"     현재 관절: {[f'{j:.1f}' for j in ret[1]]}")

# ─── RealSense 연결 ───
print("\n[RealSense 연결 중...]")
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
profile = pipeline.start(config)
align = rs.align(rs.stream.color)

# D435i depth 최적화: 레이저 파워 최대
depth_sensor = profile.get_device().first_depth_sensor()
if depth_sensor.supports(rs.option.laser_power):
    max_laser = depth_sensor.get_option_range(rs.option.laser_power).max
    depth_sensor.set_option(rs.option.laser_power, max_laser)
    print(f"  레이저 파워: {max_laser:.0f}mW (최대)")

# depth 필터 (D435i 튜닝)
spatial = rs.spatial_filter()
spatial.set_option(rs.option.filter_magnitude, 2)   # 필터 강도 (기본 2)
spatial.set_option(rs.option.filter_smooth_alpha, 0.5)  # 평활 (0.25~0.75)
spatial.set_option(rs.option.filter_smooth_delta, 20)   # 에지 보존 임계값
temporal = rs.temporal_filter()
temporal.set_option(rs.option.filter_smooth_alpha, 0.4)  # 시간 평활
hole_filling = rs.hole_filling_filter()  # depth 홀 채우기

# 안정화 (D435i auto-exposure 수렴 대기)
for _ in range(60):
    pipeline.wait_for_frames()
print("[OK] RealSense D435i 준비 완료")

# ─── 데이터 수집 ───
camera_points = []   # 카메라 3D 좌표 (mm)
robot_points = []    # 로봇 TCP 좌표 (mm)
click_pos = None
mouse_x, mouse_y = 640, 360

def on_mouse(event, x, y, flags, param):
    global mouse_x, mouse_y, click_pos
    mouse_x, mouse_y = x, y
    if event == cv2.EVENT_LBUTTONDOWN:
        click_pos = (x, y)

cv2.namedWindow('Step3 - Calibration')
cv2.setMouseCallback('Step3 - Calibration', on_mouse)

print("\n" + "-" * 60)
print("  캘리브레이션 데이터 수집")
print("-" * 60)
print("  1. 로봇 TCP(플랜지 볼트)를 다른 위치로 이동")
print("  2. 카메라 화면에서 볼트 중심을 마우스 클릭")
print("  3. Space키로 저장")
print("  4. 1~3을 반복 (최소 5회, 권장 8~10회)")
print("  5. 다 모이면 c키로 캘리브레이션 계산")
print()
print("  [팁] 포인트를 넓게 분포시켜야 정확도가 올라갑니다")
print("       한쪽에만 몰리지 않게 여러 위치에서 수집하세요")
print("-" * 60)

# 클릭 좌표 임시 저장 (Space 누르기 전)
pending_click = None  # (pixel_x, pixel_y, cam_3d)
last_errors = None    # 마지막 캘리브레이션 오차 (x키에서 사용)

def save_data_json():
    """수집된 데이터를 JSON으로 저장"""
    data = {
        'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
        'num_points': len(camera_points),
        'camera_points_mm': [p.tolist() if isinstance(p, np.ndarray) else p for p in camera_points],
        'robot_points_mm': [p.tolist() if isinstance(p, np.ndarray) else p for p in robot_points],
    }
    path = os.path.join(SAVE_DIR, 'calibration_data.json')
    with open(path, 'w') as f:
        json.dump(data, f, indent=2)
    print(f"  [저장] {path} ({len(camera_points)}개 포인트)")

def compute_calibration():
    """변환 행렬 T 계산 (카메라 mm → 로봇 mm)"""
    if len(camera_points) < 5:
        print(f"  [!] 포인트 부족 ({len(camera_points)}개). 최소 5개 필요!")
        return None, None

    cam = np.array(camera_points, dtype=np.float64)  # Nx3 (mm)
    rob = np.array(robot_points, dtype=np.float64)    # Nx3 (mm)

    # SVD 기반 rigid transform (R, t) 계산
    cam_center = cam.mean(axis=0)
    rob_center = rob.mean(axis=0)

    cam_centered = cam - cam_center
    rob_centered = rob - rob_center

    H = cam_centered.T @ rob_centered
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # reflection 보정
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = rob_center - R @ cam_center

    # 4x4 변환 행렬
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t

    # 오차 계산
    errors = []
    print("\n  ┌─────────────────────────────────────────────────┐")
    print("  │           캘리브레이션 결과                       │")
    print("  ├─────────────────────────────────────────────────┤")
    for i in range(len(cam)):
        predicted = R @ cam[i] + t
        error = np.linalg.norm(predicted - rob[i])
        errors.append(error)
        status = "OK" if error < 10 else "!!"
        print(f"  │  점 {i+1}: 오차 {error:6.1f}mm  [{status}]{'':>24}│")

    avg_err = np.mean(errors)
    max_err = np.max(errors)
    print(f"  ├─────────────────────────────────────────────────┤")
    print(f"  │  평균 오차: {avg_err:.1f}mm  /  최대 오차: {max_err:.1f}mm{'':>10}│")

    if avg_err < 5:
        grade = "우수 (5mm 이내)"
    elif avg_err < 10:
        grade = "양호 (10mm 이내)"
    elif avg_err < 20:
        grade = "보통 (20mm 이내, 재수집 권장)"
    else:
        grade = "불량 (재수집 필요)"
    print(f"  │  등급: {grade}{'':>30}│")
    print(f"  └─────────────────────────────────────────────────┘")

    # 저장
    matrix_path = os.path.join(SAVE_DIR, 'calibration_matrix.npy')
    np.save(matrix_path, T)
    print(f"\n  [저장] 변환 행렬: {matrix_path}")

    # 사람이 읽을 수 있는 형태로도 저장
    txt_path = os.path.join(SAVE_DIR, 'calibration_matrix.txt')
    with open(txt_path, 'w') as f:
        f.write(f"캘리브레이션 결과 ({datetime.now().strftime('%Y-%m-%d %H:%M:%S')})\n")
        f.write(f"포인트 수: {len(cam)}\n")
        f.write(f"평균 오차: {avg_err:.1f}mm\n")
        f.write(f"최대 오차: {max_err:.1f}mm\n\n")
        f.write("변환 행렬 T (4x4):\n")
        f.write(np.array2string(T, precision=6, suppress_small=True))
        f.write("\n\n회전 행렬 R (3x3):\n")
        f.write(np.array2string(R, precision=6, suppress_small=True))
        f.write(f"\n\n이동 벡터 t (mm): {t}\n")
        f.write(f"\n각 포인트 오차:\n")
        for i, e in enumerate(errors):
            f.write(f"  점 {i+1}: {e:.1f}mm\n")
    print(f"  [저장] 상세 결과: {txt_path}")

    return T, errors

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        # depth 필터 적용 (spatial → temporal → hole filling)
        filtered = spatial.process(depth_frame)
        filtered = temporal.process(filtered)
        filtered = hole_filling.process(filtered)
        depth_filtered = filtered.as_depth_frame()

        color_image = np.asanyarray(color_frame.get_data())
        intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
        display = color_image.copy()

        # ─── 마우스 위치 실시간 표시 ───
        depth_m = depth_filtered.get_distance(mouse_x, mouse_y)
        point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [mouse_x, mouse_y], depth_m)
        cam_mm = [p * 1000 for p in point_3d]

        cv2.drawMarker(display, (mouse_x, mouse_y), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
        cv2.putText(display, f"Camera: ({cam_mm[0]:.1f}, {cam_mm[1]:.1f}, {cam_mm[2]:.1f})mm",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(display, f"Depth: {depth_m:.3f}m",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # ─── 클릭 처리 (흰색 원 중심 자동 보정) ───
        if click_pos:
            cx, cy = click_pos

            # 클릭 주변 160x160 영역에서 빨간 마커 검출
            crop_r = 80
            x1c = max(0, cx - crop_r)
            y1c = max(0, cy - crop_r)
            x2c = min(1280, cx + crop_r)
            y2c = min(720, cy + crop_r)
            crop = color_image[y1c:y2c, x1c:x2c]

            # 빨간색 영역 검출 (HSV)
            hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
            # 빨간색은 H가 0 근처 + 170 근처 두 범위
            mask1 = cv2.inRange(hsv, (0, 30, 50), (10, 255, 255))
            mask2 = cv2.inRange(hsv, (170, 30, 50), (180, 255, 255))
            binary = mask1 | mask2
            # 모폴로지로 노이즈 제거
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
            binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

            # 윤곽선에서 원형 찾기
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            best_center = None
            best_score = 0
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 15:  # 너무 작은 건 무시
                    continue
                perimeter = cv2.arcLength(cnt, True)
                if perimeter == 0:
                    continue
                # 원형도 = 4π × 면적 / 둘레² (1에 가까울수록 원)
                circularity = 4 * np.pi * area / (perimeter * perimeter)
                if circularity > 0.5 and area > best_score:
                    M = cv2.moments(cnt)
                    if M["m00"] > 0:
                        # crop 내 좌표 → 전체 이미지 좌표
                        mcx = int(M["m10"] / M["m00"]) + x1c
                        mcy = int(M["m01"] / M["m00"]) + y1c
                        best_center = (mcx, mcy)
                        best_score = area

            if best_center:
                offset = np.sqrt((best_center[0] - cx)**2 + (best_center[1] - cy)**2)
                cx, cy = best_center
                print(f"  [보정] 빨간 마커 중심 검출 → pixel=({cx},{cy}) (보정 {offset:.1f}px)")
            else:
                print(f"  [클릭] 원 검출 실패 → 클릭 위치 그대로 사용 pixel=({cx},{cy})")

            # 5x5 영역 depth 중앙값 (단일 픽셀 노이즈 방지)
            radius = 2
            depths = []
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    px, py = cx + dx, cy + dy
                    if 0 <= px < 1280 and 0 <= py < 720:
                        dd = depth_filtered.get_distance(px, py)
                        if dd > 0.2:
                            depths.append(dd)
            if len(depths) < 5:
                print(f"  [!] 유효 depth 부족 ({len(depths)}개) → 다시 클릭하세요")
            else:
                d = float(np.median(depths))
                p = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], d)
                pending_click = (cx, cy, [p[0]*1000, p[1]*1000, p[2]*1000])
                print(f"  [클릭] pixel=({cx},{cy}) depth={d:.4f}m (5x5 median, {len(depths)}개)")
                print(f"         cam=({p[0]*1000:.1f}, {p[1]*1000:.1f}, {p[2]*1000:.1f})mm")
                print(f"         → Space키로 저장 (로봇 TCP 좌표 동시 기록)")
            click_pos = None

        # ─── 클릭 대기 중 표시 ───
        if pending_click:
            px, py, _ = pending_click
            cv2.circle(display, (px, py), 12, (0, 255, 255), 3)
            cv2.putText(display, "Space to save", (px + 15, py + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # ─── 저장된 포인트 표시 ───
        for i in range(len(camera_points)):
            # 카메라 좌표를 다시 픽셀로 변환하기 어려우므로 원본 정보 없음
            # 대신 상단에 리스트로 표시
            pass

        # 상태 바
        n = len(camera_points)
        status_color = (0, 255, 0) if n >= 5 else (0, 165, 255)
        cv2.putText(display, f"Points: {n}/5+", (10, 700),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
        cv2.putText(display, "Click|Space:save|c:calc|x:remove>10mm|z:delete(#)|s:save|q:quit",
                    (200, 700), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        # 저장된 포인트 목록 (우측 상단)
        for i in range(min(len(camera_points), 12)):
            c = camera_points[i]
            r = robot_points[i]
            cv2.putText(display, f"#{i+1} cam({c[0]:.0f},{c[1]:.0f},{c[2]:.0f}) rob({r[0]:.0f},{r[1]:.0f},{r[2]:.0f})",
                        (700, 30 + i * 22), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

        cv2.imshow('Step3 - Calibration', display)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break

        elif key == ord(' ') and pending_click:
            # Space: 카메라 좌표 + 로봇 TCP 좌표 동시 저장
            _, _, cam_3d = pending_click
            ret = robot.Get_Current_Arm_State(retry=1)
            # ret[2] = [x, y, z, rx, ry, rz] (미터, 라디안)
            tcp = ret[2]

            if tcp is None or len(tcp) < 3:
                print("  [!] TCP 좌표를 가져올 수 없습니다")
                print(f"      ret = {ret}")
            else:
                # 미터 → 밀리미터 변환
                tcp_xyz = [tcp[0] * 1000, tcp[1] * 1000, tcp[2] * 1000]
                camera_points.append(cam_3d)
                robot_points.append(tcp_xyz)
                n = len(camera_points)
                print(f"  [#{n} 저장] cam=({cam_3d[0]:.1f}, {cam_3d[1]:.1f}, {cam_3d[2]:.1f})mm "
                      f"| robot=({tcp_xyz[0]:.1f}, {tcp_xyz[1]:.1f}, {tcp_xyz[2]:.1f})mm")
                if n < 5:
                    print(f"         → {5-n}개 더 필요 (최소 5개)")
                else:
                    print(f"         → c키로 캘리브레이션 계산 가능 ({n}개)")
                pending_click = None

        elif key == ord('c'):
            # 캘리브레이션 계산
            T, last_errors = compute_calibration()
            if T is not None:
                save_data_json()
                print("  → x키: 오차 10mm 이상 포인트 자동 삭제 후 재계산")

        elif key == ord('x'):
            # 오차 10mm 이상 포인트 자동 삭제 후 재계산
            if last_errors is None:
                print("  [!] 먼저 c키로 캘리브레이션을 계산하세요")
            elif len(camera_points) < 5:
                print("  [!] 포인트 부족")
            else:
                # 오차 큰 포인트 찾기 (역순으로 삭제해야 인덱스 안 꼬임)
                bad_indices = [i for i, e in enumerate(last_errors) if e >= 10]
                if not bad_indices:
                    print("  [OK] 10mm 이상 오차 포인트 없음!")
                else:
                    for idx in sorted(bad_indices, reverse=True):
                        c = camera_points.pop(idx)
                        r = robot_points.pop(idx)
                        print(f"  [삭제] #{idx+1} 오차 {last_errors[idx]:.1f}mm: cam=({c[0]:.0f},{c[1]:.0f},{c[2]:.0f})")
                    print(f"  → {len(bad_indices)}개 삭제, {len(camera_points)}개 남음")
                    if len(camera_points) < 5:
                        print(f"  [!] {5 - len(camera_points)}개 더 찍어야 재계산 가능")
                        last_errors = None
                    else:
                        print("  재계산 중...")
                        T, last_errors = compute_calibration()
                        if T is not None:
                            save_data_json()

        elif key == ord('z'):
            # 포인트 삭제 (번호 지정)
            if not camera_points:
                print("  [!] 삭제할 포인트가 없습니다")
            else:
                print(f"\n  현재 포인트 목록 ({len(camera_points)}개):")
                for i in range(len(camera_points)):
                    c = camera_points[i]
                    r = robot_points[i]
                    print(f"    #{i+1} cam=({c[0]:.0f},{c[1]:.0f},{c[2]:.0f}) rob=({r[0]:.0f},{r[1]:.0f},{r[2]:.0f})")
                try:
                    num = input("  삭제할 번호 (Enter=마지막, 0=취소): ").strip()
                    if num == '0':
                        print("  [취소]")
                    elif num == '':
                        camera_points.pop()
                        robot_points.pop()
                        print(f"  [삭제] 마지막 포인트 제거 (남은 {len(camera_points)}개)")
                    else:
                        idx = int(num) - 1
                        if 0 <= idx < len(camera_points):
                            c = camera_points.pop(idx)
                            r = robot_points.pop(idx)
                            print(f"  [삭제] #{num} 제거: cam=({c[0]:.0f},{c[1]:.0f},{c[2]:.0f}) (남은 {len(camera_points)}개)")
                        else:
                            print(f"  [!] 잘못된 번호 (1~{len(camera_points)})")
                except ValueError:
                    print("  [!] 숫자를 입력하세요")
            pending_click = None

        elif key == ord('s'):
            # 중간 저장
            save_data_json()

finally:
    pipeline.stop()
    robot.Arm_Socket_Close()
    cv2.destroyAllWindows()

    if camera_points:
        save_data_json()
        print(f"\n  최종 저장 완료 ({len(camera_points)}개 포인트)")

    print("[OK] 종료")
