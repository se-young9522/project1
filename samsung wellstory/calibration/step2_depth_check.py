"""
2단계: 카메라 고정 + Depth 정확도 확인
- 카메라를 삼각대/클램프로 고정한 뒤 실행
- 마우스 클릭한 위치의 3D 좌표를 기록
- 여러 위치에서 depth 값이 안정적인지 확인
- 실제 거리(줄자)와 비교하여 오차 확인

사용법: python step2_depth_check.py
  마우스 클릭: 해당 위치의 3D 좌표 기록
  r: 기록 초기화
  q: 종료
"""
import sys
import numpy as np
import cv2
import pyrealsense2 as rs

# ─── RealSense 초기화 ───
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

profile = pipeline.start(config)
align = rs.align(rs.stream.color)

# depth 필터 (노이즈 감소)
spatial = rs.spatial_filter()       # 공간 필터: 주변 픽셀 평균
temporal = rs.temporal_filter()     # 시간 필터: 이전 프레임과 평균
hole_fill = rs.hole_filling_filter()  # 빈 구멍 채우기

print("=" * 50)
print("  2단계: Depth 정확도 확인")
print("=" * 50)
print()
print("  [준비사항]")
print("  1. 카메라를 삼각대/클램프로 고정")
print("  2. 작업 영역(테이블)이 잘 보이도록 설치")
print("  3. 카메라 ~ 테이블 거리: 30cm 이상")
print()
print("  [사용법]")
print("  마우스 클릭: 해당 위치의 3D 좌표 기록")
print("  a: 카메라 기울기 각도 자동 측정")
print("  r: 기록 초기화")
print("  q: 종료")
print("=" * 50)

# ─── 클릭 데이터 저장 ───
click_points = []   # [(pixel_x, pixel_y, X, Y, Z), ...]
mouse_x, mouse_y = 640, 360
click_pos = None

def on_mouse(event, x, y, flags, param):
    global mouse_x, mouse_y, click_pos
    mouse_x, mouse_y = x, y
    if event == cv2.EVENT_LBUTTONDOWN:
        click_pos = (x, y)

cv2.namedWindow('Step2 - Depth Check')
cv2.setMouseCallback('Step2 - Depth Check', on_mouse)

# depth 안정화를 위해 처음 30프레임 버리기
print("\n  카메라 안정화 중...")
for _ in range(30):
    pipeline.wait_for_frames()
print("  준비 완료!\n")

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        # depth 필터 적용 (노이즈 감소)
        filtered_depth = spatial.process(depth_frame)
        filtered_depth = temporal.process(filtered_depth)
        filtered_depth = hole_fill.process(filtered_depth)

        color_image = np.asanyarray(color_frame.get_data())
        intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

        display = color_image.copy()

        # ─── 마우스 위치 실시간 표시 ───
        depth_m = filtered_depth.as_depth_frame().get_distance(mouse_x, mouse_y)
        point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [mouse_x, mouse_y], depth_m)

        # 십자선
        cv2.drawMarker(display, (mouse_x, mouse_y), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)

        # 실시간 정보
        cv2.putText(display, f"Pixel: ({mouse_x}, {mouse_y})  Depth: {depth_m:.3f}m ({depth_m*1000:.0f}mm)",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(display, f"3D: X={point_3d[0]*1000:.1f}mm  Y={point_3d[1]*1000:.1f}mm  Z={point_3d[2]*1000:.1f}mm",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # ─── 클릭 시 좌표 기록 ───
        if click_pos:
            cx, cy = click_pos
            d = filtered_depth.as_depth_frame().get_distance(cx, cy)
            p = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], d)

            if d < 0.1:
                print(f"  [!] depth={d:.3f}m → 너무 가까움 (최소 30cm 이상)")
            else:
                click_points.append((cx, cy, p[0], p[1], p[2]))
                idx = len(click_points)
                print(f"  #{idx}: pixel=({cx},{cy})  depth={d:.3f}m  "
                      f"3D=({p[0]*1000:.1f}, {p[1]*1000:.1f}, {p[2]*1000:.1f})mm")

            click_pos = None

        # ─── 기록된 포인트들 화면에 표시 ───
        for i, (px, py, X, Y, Z) in enumerate(click_points):
            cv2.circle(display, (px, py), 8, (0, 0, 255), 2)
            cv2.putText(display, f"#{i+1} ({Z*1000:.0f}mm)",
                        (px + 10, py - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # 두 점 사이 거리 표시 (마지막 2개)
        if len(click_points) >= 2:
            p1 = np.array(click_points[-2][2:5])  # X, Y, Z
            p2 = np.array(click_points[-1][2:5])
            dist = np.linalg.norm(p1 - p2) * 1000  # mm

            # 화면에 선 + 거리
            pt1 = (click_points[-2][0], click_points[-2][1])
            pt2 = (click_points[-1][0], click_points[-1][1])
            cv2.line(display, pt1, pt2, (255, 0, 255), 2)
            mid = ((pt1[0]+pt2[0])//2, (pt1[1]+pt2[1])//2)
            cv2.putText(display, f"{dist:.1f}mm", (mid[0]+10, mid[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)

        # 안내 텍스트
        cv2.putText(display, f"Points: {len(click_points)}  |  Click: record  |  a: angle  |  r: reset  |  q: quit",
                    (10, 700), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        cv2.imshow('Step2 - Depth Check', display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('a'):
            # 카메라 기울기 각도 측정: 클릭한 2개 이상의 테이블 점으로 계산
            if len(click_points) < 3:
                print(f"\n  [!] 테이블 위 빈 곳을 3개 이상 클릭한 뒤 a키를 누르세요")
                print(f"      (가능하면 멀리 떨어진 점 3~5개, 현재 {len(click_points)}개)")
            else:
                # 클릭한 점들의 3D 좌표로 평면 fitting
                pts = np.array([[p[2], p[3], p[4]] for p in click_points])  # X, Y, Z (meters)
                centroid = pts.mean(axis=0)
                pts_c = pts - centroid
                _, S, Vt = np.linalg.svd(pts_c)
                normal = Vt[2]
                if normal[2] < 0:
                    normal = -normal

                # 카메라 광축 [0,0,1]과 테이블 법선 사이의 각도
                cos_angle = abs(np.dot(normal, [0, 0, 1]))
                angle_deg = np.degrees(np.arccos(np.clip(cos_angle, 0, 1)))

                # 점들 간 최대 거리 (측정 신뢰도 확인)
                max_dist = 0
                for ii in range(len(pts)):
                    for jj in range(ii+1, len(pts)):
                        d = np.linalg.norm(pts[ii] - pts[jj])
                        max_dist = max(max_dist, d)

                print(f"  ┌─────────────────────────────────")
                print(f"  │ 카메라 기울기 각도: {angle_deg:.1f}° (수직 기준)")
                print(f"  │ 법선 벡터: ({normal[0]:.4f}, {normal[1]:.4f}, {normal[2]:.4f})")
                print(f"  │ 사용한 점: {len(click_points)}개")
                print(f"  │ 점 간 최대 거리: {max_dist*1000:.0f}mm")
                if max_dist < 0.1:
                    print(f"  │ ⚠ 점들이 너무 가까움! 더 넓게 클릭하세요")
                if angle_deg < 5:
                    print(f"  │ → 거의 수직 (이상적)")
                elif angle_deg < 15:
                    print(f"  │ → 약간 기울어짐 (보정 권장)")
                elif angle_deg < 30:
                    print(f"  │ → 상당히 기울어짐 (보정 필요)")
                else:
                    print(f"  │ → 많이 기울어짐 (카메라 재설치 권장)")
                print(f"  └─────────────────────────────────")
                print(f"  ※ r키로 리셋 후 다른 위치에서도 해보세요")

        elif key == ord('r'):
            click_points.clear()
            print("\n  [리셋] 기록 초기화\n")

finally:
    pipeline.stop()
    cv2.destroyAllWindows()

    # ─── 결과 요약 ───
    if click_points:
        print("\n" + "=" * 50)
        print("  기록된 포인트 요약")
        print("=" * 50)
        print(f"  {'#':<5}{'Pixel':<16}{'X(mm)':<10}{'Y(mm)':<10}{'Z(mm)':<10}")
        print(f"  {'---':<5}{'----------':<16}{'------':<10}{'------':<10}{'------':<10}")
        for i, (px, py, X, Y, Z) in enumerate(click_points):
            print(f"  {i+1:<5}({px},{py}){'':<6}{X*1000:<10.1f}{Y*1000:<10.1f}{Z*1000:<10.1f}")

        # 같은 위치를 여러번 클릭했다면 반복성 확인
        if len(click_points) >= 2:
            zs = [p[4] for p in click_points]
            print(f"\n  Depth 범위: {min(zs)*1000:.1f} ~ {max(zs)*1000:.1f}mm")
            print(f"  Depth 편차: {np.std(zs)*1000:.1f}mm")
    else:
        print("\n  기록된 포인트 없음")

    print("\n[OK] 종료 완료")
