"""
1단계: RealSense + 로봇 연결 확인
- RealSense 컬러/depth 스트림 확인
- 로봇 연결 + 현재 TCP 좌표/관절각도 확인
- 화면에 depth 값 실시간 표시 (마우스 위치)

사용법: python step1_check_connection.py
종료: q키
"""
import sys
import numpy as np
import cv2

# ─── 1-1. RealSense 연결 확인 ───
print("=" * 50)
print("  1단계: 연결 확인")
print("=" * 50)

try:
    import pyrealsense2 as rs
    print("[OK] pyrealsense2 import 성공")
except ImportError:
    print("[FAIL] pyrealsense2가 설치되지 않았습니다")
    print("  → python -m pip install pyrealsense2")
    sys.exit(1)

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

try:
    profile = pipeline.start(config)
    device = profile.get_device()
    print(f"[OK] RealSense 연결 성공")
    print(f"     모델: {device.get_info(rs.camera_info.name)}")
    print(f"     시리얼: {device.get_info(rs.camera_info.serial_number)}")
    print(f"     펌웨어: {device.get_info(rs.camera_info.firmware_version)}")
except Exception as e:
    print(f"[FAIL] RealSense 연결 실패: {e}")
    print("  → USB 케이블 확인 (USB 3.0 필수)")
    sys.exit(1)

align = rs.align(rs.stream.color)

# depth intrinsics 확인
frames = pipeline.wait_for_frames()
aligned = align.process(frames)
depth_frame = aligned.get_depth_frame()
intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
print(f"     해상도: {intrinsics.width}x{intrinsics.height}")
print(f"     초점거리: fx={intrinsics.fx:.1f}, fy={intrinsics.fy:.1f}")
print(f"     중심점: cx={intrinsics.ppx:.1f}, cy={intrinsics.ppy:.1f}")

# ─── 1-2. 로봇 연결 확인 ───
print()
robot = None
robot_connected = False

try:
    sys.path.insert(0, r'C:\Users\ASUS\Desktop\samsung wellstory\RM_API-main\Example\API_Example_Python\API_Example_Python')
    from robotic_arm_package.robotic_arm import *
    print("[OK] robotic_arm import 성공")
except ImportError:
    print("[WARN] robotic_arm import 실패 (로봇 없이 카메라만 테스트)")

if 'Arm' in dir():
    try:
        robot = Arm(RM65, "192.168.1.18")
        print(f"[OK] 로봇 연결 성공 (192.168.1.18)")
        print(f"     API 버전: {robot.API_Version()}")

        ret = robot.Get_Current_Arm_State(retry=1)
        joint = ret[1]  # 관절 각도 [j1~j6]
        # TCP 좌표는 ret[2] 또는 별도 호출
        print(f"     관절각도: {[f'{j:.1f}' for j in joint]}")

        robot_connected = True
    except Exception as e:
        print(f"[FAIL] 로봇 연결 실패: {e}")
        print("  → ping 192.168.1.18 확인")
        print("  → 이더넷 IP가 192.168.1.100인지 확인")

# ─── 1-3. 실시간 화면 표시 ───
print()
print("-" * 50)
print("  실시간 미리보기 시작")
print("  마우스를 움직이면 해당 위치의 depth 값 표시")
print("  q: 종료")
print("-" * 50)

mouse_x, mouse_y = 640, 360

def on_mouse(event, x, y, flags, param):
    global mouse_x, mouse_y
    if event == cv2.EVENT_MOUSEMOVE:
        mouse_x, mouse_y = x, y

cv2.namedWindow('Step1 - Connection Check')
cv2.setMouseCallback('Step1 - Connection Check', on_mouse)

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # depth 컬러맵 (시각화용)
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03),
            cv2.COLORMAP_JET
        )

        # 마우스 위치의 depth 값
        depth_m = depth_frame.get_distance(mouse_x, mouse_y)

        # 마우스 위치 → 카메라 3D 좌표
        intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
        point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [mouse_x, mouse_y], depth_m)

        # 화면에 정보 표시
        display = color_image.copy()

        # 마우스 십자선
        cv2.drawMarker(display, (mouse_x, mouse_y), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)

        # depth 값
        info_y = 30
        cv2.putText(display, f"Pixel: ({mouse_x}, {mouse_y})", (10, info_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        info_y += 30
        cv2.putText(display, f"Depth: {depth_m:.3f}m ({depth_m*1000:.0f}mm)", (10, info_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        info_y += 30
        cv2.putText(display, f"Camera 3D: X={point_3d[0]:.3f} Y={point_3d[1]:.3f} Z={point_3d[2]:.3f} (m)",
                    (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # 로봇 상태 (연결된 경우)
        if robot_connected:
            info_y += 30
            cv2.putText(display, f"Robot: Connected (192.168.1.18)", (10, info_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        else:
            info_y += 30
            cv2.putText(display, f"Robot: Not connected", (10, info_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # 안내
        cv2.putText(display, "q: quit", (10, 700),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # 컬러 + depth 나란히 표시
        depth_resized = cv2.resize(depth_colormap, (320, 180))
        display[10:190, 940:1260] = depth_resized
        cv2.rectangle(display, (940, 10), (1260, 190), (255, 255, 255), 1)
        cv2.putText(display, "Depth Map", (945, 205),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.imshow('Step1 - Connection Check', display)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    if robot and robot_connected:
        robot.Arm_Socket_Close()
        print("\n[OK] 로봇 연결 해제")
    cv2.destroyAllWindows()
    print("[OK] 종료 완료")
