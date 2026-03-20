"""
Notion - RealMan 페이지 하위에 'RealSense + 로봇팔 객체인식 Pick & Place 단계별 확인' 페이지 생성
"""
import json, urllib.request, sys, time
sys.stdout.reconfigure(encoding='utf-8')

API_KEY = 'ntn_b57077336502qA3Deg7jq4qWxqnc8navEmCPcNhjyv7gn7'
RM_PAGE_ID = '32115435-1a92-81d0-816a-f016c4cecf14'

headers = {
    'Authorization': f'Bearer {API_KEY}',
    'Notion-Version': '2022-06-28',
    'Content-Type': 'application/json',
}

def api(url, data=None, method='GET'):
    req = urllib.request.Request(
        url,
        data=json.dumps(data).encode('utf-8') if data else None,
        headers=headers,
        method=method,
    )
    try:
        resp = urllib.request.urlopen(req)
        return json.loads(resp.read().decode('utf-8'))
    except urllib.error.HTTPError as e:
        body = e.read().decode('utf-8')
        print(f'ERROR {e.code}: {body[:500]}')
        raise

def text(content, bold=False, italic=False, color='default', code=False):
    annotations = {}
    if bold: annotations['bold'] = True
    if italic: annotations['italic'] = True
    if color != 'default': annotations['color'] = color
    if code: annotations['code'] = True
    return {'type': 'text', 'text': {'content': content}, 'annotations': annotations} if annotations else {'type': 'text', 'text': {'content': content}}

def heading(level, content):
    key = f'heading_{level}'
    return {'object': 'block', 'type': key, key: {'rich_text': [text(content)]}}

def paragraph(*rich_texts):
    return {'object': 'block', 'type': 'paragraph', 'paragraph': {'rich_text': list(rich_texts)}}

def bullet(content):
    return {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {'rich_text': [text(content)]}}

def numbered(content):
    return {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {'rich_text': [text(content)]}}

def code_block(content, lang='python'):
    if len(content) > 2000:
        content = content[:1997] + '...'
    return {'object': 'block', 'type': 'code', 'code': {
        'rich_text': [text(content)], 'language': lang}}

def divider():
    return {'object': 'block', 'type': 'divider', 'divider': {}}

def callout(content, emoji='📌'):
    return {'object': 'block', 'type': 'callout', 'callout': {
        'rich_text': [text(content)],
        'icon': {'type': 'emoji', 'emoji': emoji},
    }}

def table_row(cells):
    return {'object': 'block', 'type': 'table_row', 'table_row': {
        'cells': [[text(c)] for c in cells]
    }}

def table(width, rows):
    return {'object': 'block', 'type': 'table', 'table': {
        'table_width': width,
        'has_column_header': True,
        'has_row_header': False,
        'children': [table_row(r) for r in rows],
    }}

def to_do(content, checked=False):
    return {'object': 'block', 'type': 'to_do', 'to_do': {
        'rich_text': [text(content)],
        'checked': checked,
    }}

# ──────────────────────────────────────────────────────────────
# 하위 페이지 생성
# ──────────────────────────────────────────────────────────────
print('하위 페이지 생성 중...')
new_page = api('https://api.notion.com/v1/pages', {
    'parent': {'page_id': RM_PAGE_ID},
    'properties': {
        'title': {
            'title': [text('RealSense + 로봇팔 객체인식 Pick & Place 단계별 확인')]
        }
    },
    'icon': {'type': 'emoji', 'emoji': '🎯'},
}, method='POST')

PAGE_ID = new_page['id']
print(f'  페이지 생성 완료: {PAGE_ID}')

def append(children):
    for i in range(0, len(children), 100):
        batch = children[i:i+100]
        api(f'https://api.notion.com/v1/blocks/{PAGE_ID}/children', {
            'children': batch
        }, method='PATCH')
        time.sleep(0.3)

# ══════════════════════════════════════════════════════════════
# 개요
# ══════════════════════════════════════════════════════════════
print('개요 작성...')
append([
    callout('RealSense 카메라(로봇팔 장착) + YOLOv8 Seg + RM65 로봇팔로 bowl Pick & Place 구현을 위한 단계별 확인 가이드', '🦾'),
    paragraph(text('작성일: 2026-03-13 | 현재 보유: ', bold=True), text('bowl_seg_v53 모델 (~5000장), RealSense D435, RM65 로봇팔')),
    divider(),
])

# ══════════════════════════════════════════════════════════════
# 1단계: 개별 장비 동작 확인
# ══════════════════════════════════════════════════════════════
print('1단계: 개별 장비 동작 확인...')
append([
    heading(1, '1단계: 개별 장비 동작 확인'),
    callout('각 장비가 독립적으로 정상 동작하는지 먼저 확인. 문제 발생 시 원인 분리가 쉬움.', '💡'),
])

# 1-1 로봇팔
append([
    heading(2, '1-1. 로봇팔 연결 테스트'),
    to_do('PC ↔ 로봇 네트워크 연결 확인 (ping 192.168.1.18)'),
    to_do('SDK 연결 → API 버전 출력'),
    to_do('현재 관절각도 / TCP 좌표 조회 (demo2)'),
    to_do('간단한 Movej 이동 테스트 (저속 v=10)'),
    to_do('그리퍼 열기/닫기 테스트'),
])

code_test_robot = """from robotic_arm_package.robotic_arm import *

robot = Arm(RM65, "192.168.1.18")
print("API 버전:", robot.API_Version())

# 현재 상태 조회
ret = robot.Get_Current_Arm_State(retry=1)
print("관절각도:", ret[1])

# 간단한 이동 테스트 (저속!)
zero = [0, 0, 0, 0, 0, 0]
robot.Movej_Cmd(zero, v=10, r=0)

robot.Arm_Socket_Close()"""

append([
    code_block(code_test_robot),
    paragraph(text('⚠️ 첫 이동 테스트는 반드시 비상정지 버튼 손에 쥐고, v=10 이하로!', color='red', bold=True)),
])

# 1-2 RealSense
append([
    heading(2, '1-2. RealSense 카메라 테스트'),
    to_do('USB 연결 → realsense-viewer에서 RGB/Depth 스트림 확인'),
    to_do('로봇팔 마운트 상태에서 작업 영역이 화면에 보이는지 확인'),
    to_do('pyrealsense2로 프레임 획득 테스트'),
    to_do('컬러 + Depth 동시 스트림 정상 출력'),
])

code_test_rs = """import pyrealsense2 as rs
import numpy as np
import cv2

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
pipeline.start(config)
align = rs.align(rs.stream.color)

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)
        color = np.asanyarray(aligned.get_color_frame().get_data())
        depth = np.asanyarray(aligned.get_depth_frame().get_data())

        # depth를 컬러맵으로 시각화
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET)

        cv2.imshow('Color', color)
        cv2.imshow('Depth', depth_colormap)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()"""

append([code_block(code_test_rs)])

# 1-3 YOLO
append([
    heading(2, '1-3. YOLO Seg 추론 테스트'),
    to_do('RealSense 컬러 프레임에 bowl_seg_v53 모델 추론'),
    to_do('bowl 마스크 정상 검출 확인 (conf 0.7+)'),
    to_do('다양한 위치/각도에서 검출 안정성 확인'),
])

code_test_yolo = """from ultralytics import YOLO
import pyrealsense2 as rs
import numpy as np
import cv2

model = YOLO(r'C:\\Users\\ASUS\\runs\\segment\\bowl_seg_v53\\weights\\best.pt')

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)

try:
    while True:
        frames = pipeline.wait_for_frames()
        color = np.asanyarray(frames.get_color_frame().get_data())

        results = model(color, conf=0.7)
        annotated = results[0].plot()

        cv2.imshow('YOLO Seg', annotated)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()"""

append([
    code_block(code_test_yolo),
    divider(),
])

# ══════════════════════════════════════════════════════════════
# 2단계: 카메라 좌표 → 3D 좌표 변환
# ══════════════════════════════════════════════════════════════
print('2단계: 3D 좌표 변환...')
append([
    heading(1, '2단계: 카메라 좌표 → 3D 좌표 변환'),
    callout('Detection 결과(픽셀)를 실제 3D 좌표로 변환. 캘리브레이션 전에 depth 값이 안정적인지 먼저 확인.', '📏'),
])

append([
    heading(2, '2-1. Depth 값 읽기 & 3D 변환'),
    to_do('detection된 bowl 중심점(cx, cy)에서 depth 값 읽기'),
    to_do('rs2_deproject_pixel_to_point로 카메라 3D 좌표 얻기'),
    to_do('여러 위치에서 depth 값 합리성 확인 (0.3~0.8m 범위)'),
    to_do('같은 위치 반복 측정 → 값이 안정적인지 확인 (±5mm 이내)'),
])

code_depth = """import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO

model = YOLO(r'C:\\Users\\ASUS\\runs\\segment\\bowl_seg_v53\\weights\\best.pt')

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
pipeline.start(config)
align = rs.align(rs.stream.color)

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        color_image = np.asanyarray(color_frame.get_data())
        intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

        results = model(color_image, conf=0.7)

        for r in results:
            if r.masks is None:
                continue
            for mask in r.masks.xy:
                # 마스크 중심점 계산
                cx = int(mask[:, 0].mean())
                cy = int(mask[:, 1].mean())

                # depth 값 읽기
                depth = depth_frame.get_distance(cx, cy)

                # 카메라 3D 좌표 변환
                point_3d = rs.rs2_deproject_pixel_to_point(
                    intrinsics, [cx, cy], depth)

                # 화면에 표시
                cv2.circle(color_image, (cx, cy), 5, (0, 255, 0), -1)
                info = f"({point_3d[0]:.3f}, {point_3d[1]:.3f}, {point_3d[2]:.3f})m"
                cv2.putText(color_image, info, (cx+10, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                print(f"픽셀({cx},{cy}) depth={depth:.3f}m 3D={point_3d}")

        cv2.imshow('3D Coordinate Test', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()"""

append([code_block(code_depth)])

append([
    heading(2, '2-2. Depth 안정성 체크 포인트'),
    table(2, [
        ['확인 항목', '기준'],
        ['depth 범위', '0.3~0.8m (너무 가까우면 0, 너무 멀면 노이즈)'],
        ['반복 측정 오차', '같은 위치 10회 측정 → ±5mm 이내'],
        ['빈 depth (0값)', '중심점이 hole에 걸리면 주변 평균 사용'],
        ['반사 표면', '스테인리스 그릇은 IR 반사로 depth 불안정 가능'],
    ]),
    callout('depth가 0이면: 중심점 주변 5x5 영역의 중앙값(median)을 사용하여 보정', '💡'),
    divider(),
])

# ══════════════════════════════════════════════════════════════
# 3단계: Hand-Eye Calibration
# ══════════════════════════════════════════════════════════════
print('3단계: Hand-Eye Calibration...')
append([
    heading(1, '3단계: Hand-Eye Calibration (핵심!)'),
    callout('카메라가 로봇팔에 장착 → Eye-in-Hand 방식. 카메라 좌표 → 로봇 베이스 좌표 변환 행렬(T) 구하기', '🔑'),
    paragraph(text('⏱ 이 단계가 가장 시간이 오래 걸리고, 전체 정확도를 좌우하는 핵심 단계', color='red', bold=True)),
])

append([
    heading(2, '3-1. Eye-in-Hand vs Eye-to-Hand'),
    table(3, [
        ['구분', 'Eye-in-Hand (현재)', 'Eye-to-Hand'],
        ['카메라 위치', '로봇팔 끝에 장착', '고정 위치 (외부)'],
        ['변환 관계', 'T_cam→tcp (카메라↔TCP)', 'T_cam→base (카메라↔베이스)'],
        ['특징', '팔 움직이면 카메라도 이동', '카메라 고정, 로봇만 이동'],
        ['장점', '가까이 관찰 가능, 정밀', '캘리브레이션 1회'],
        ['단점', '매 촬영마다 TCP 포즈 필요', '거리에 따라 정밀도 저하'],
    ]),
])

append([
    heading(2, '3-2. 캘리브레이션 데이터 수집'),
    to_do('ArUco 마커 또는 체커보드를 작업대 위 고정 위치에 놓기'),
    to_do('로봇팔을 5~10개 다른 자세로 이동'),
    to_do('각 자세에서: 로봇 TCP 좌표 기록 + 카메라로 마커 감지'),
    to_do('최소 5쌍, 권장 8~10쌍 수집'),
])

code_collect_calib = """import pyrealsense2 as rs
import numpy as np
import cv2
from robotic_arm_package.robotic_arm import *

# ─── 로봇 연결 ───
robot = Arm(RM65, "192.168.1.18")

# ─── RealSense 초기화 ───
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)

# ─── ArUco 설정 ───
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# ─── 데이터 수집 ───
R_gripper2base_list = []  # 로봇 TCP → 베이스 변환 (회전)
t_gripper2base_list = []  # 로봇 TCP → 베이스 변환 (이동)
R_target2cam_list = []    # 마커 → 카메라 변환 (회전)
t_target2cam_list = []    # 마커 → 카메라 변환 (이동)

print("로봇을 다른 자세로 이동시킨 후 Enter 키를 눌러 데이터 수집")
print("최소 5회, 권장 8~10회. 종료: q")

count = 0
while True:
    # 카메라 프레임
    frames = pipeline.wait_for_frames()
    color = np.asanyarray(frames.get_color_frame().get_data())
    gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)

    # ArUco 감지
    corners, ids, _ = detector.detectMarkers(gray)
    if corners:
        cv2.aruco.drawDetectedMarkers(color, corners, ids)

    cv2.imshow('Calibration', color)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        break
    elif key == 13:  # Enter
        if not corners:
            print("  마커가 감지되지 않음! 다시 시도")
            continue

        # 로봇 TCP 포즈 가져오기
        ret = robot.Get_Current_Arm_State(retry=1)
        tcp = ret[1]  # [x, y, z, rx, ry, rz]
        print(f"  수집 {count+1}: TCP = {tcp}")

        # 마커 포즈 추정 (카메라 intrinsics 필요)
        # TODO: 실제 마커 크기, 카메라 intrinsics로 solvePnP
        # rvec, tvec = 마커 포즈

        count += 1
        print(f"  총 {count}개 수집됨")

pipeline.stop()
cv2.destroyAllWindows()
print(f"\\n수집 완료: {count}개 포즈")"""

append([code_block(code_collect_calib)])

append([
    heading(2, '3-3. 변환 행렬 계산'),
    paragraph(text('OpenCV '), text('cv2.calibrateHandEye()', code=True), text(' 사용')),
])

code_handeye = """import cv2
import numpy as np

# 수집한 데이터로 Hand-Eye Calibration
# R_gripper2base: 각 자세에서 TCP의 회전 행렬
# t_gripper2base: 각 자세에서 TCP의 이동 벡터
# R_target2cam: 각 자세에서 마커→카메라 회전 행렬
# t_target2cam: 각 자세에서 마커→카메라 이동 벡터

R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
    R_gripper2base_list,
    t_gripper2base_list,
    R_target2cam_list,
    t_target2cam_list,
    method=cv2.CALIB_HAND_EYE_TSAI  # 또는 PARK, HORAUD 등
)

# 4x4 변환 행렬로 저장
T_cam2tcp = np.eye(4)
T_cam2tcp[:3, :3] = R_cam2gripper
T_cam2tcp[:3, 3] = t_cam2gripper.flatten()
np.save('T_cam2tcp.npy', T_cam2tcp)
print("캘리브레이션 완료!")
print(f"T_cam2tcp:\\n{T_cam2tcp}")"""

append([code_block(code_handeye)])

append([
    heading(2, '3-4. 좌표 변환 공식 (Eye-in-Hand)'),
    callout('P_base = T_tcp2base × T_cam2tcp × P_camera', '🔄'),
    bullet('P_camera: RealSense에서 얻은 카메라 3D 좌표 (미터)'),
    bullet('T_cam2tcp: 캘리브레이션으로 구한 행렬 (카메라→TCP)'),
    bullet('T_tcp2base: 현재 로봇 TCP 포즈로부터 계산 (TCP→베이스)'),
    bullet('P_base: 최종 로봇 베이스 좌표 → movel 명령에 사용'),
])

code_transform = """def camera_to_base(cam_point, T_cam2tcp, robot_tcp_pose):
    \"\"\"
    카메라 3D 좌표 → 로봇 베이스 좌표 변환
    cam_point: [X, Y, Z] 카메라 좌표 (미터)
    T_cam2tcp: 캘리브레이션 행렬 (4x4)
    robot_tcp_pose: 현재 TCP [x,y,z,rx,ry,rz] (mm, rad)
    \"\"\"
    # TCP → Base 변환 행렬 구성
    x, y, z, rx, ry, rz = robot_tcp_pose
    R_tcp2base, _ = cv2.Rodrigues(np.array([rx, ry, rz]))
    T_tcp2base = np.eye(4)
    T_tcp2base[:3, :3] = R_tcp2base
    T_tcp2base[:3, 3] = [x, y, z]  # mm

    # 카메라 좌표 → mm 변환
    p_cam = np.array([*cam_point, 1.0])
    p_cam[:3] *= 1000  # m → mm

    # 변환: cam → tcp → base
    p_base = T_tcp2base @ T_cam2tcp @ p_cam
    return p_base[:3]  # [X, Y, Z] mm"""

append([
    code_block(code_transform),
    divider(),
])

# ══════════════════════════════════════════════════════════════
# 4단계: 통합 테스트 (수동)
# ══════════════════════════════════════════════════════════════
print('4단계: 통합 테스트...')
append([
    heading(1, '4단계: 통합 테스트 (수동 검증)'),
    callout('실제 로봇을 움직이기 전에 좌표 변환 정확도를 먼저 확인!', '🧪'),
])

append([
    heading(2, '4-1. "보고 → 좌표 출력"만 먼저'),
    to_do('카메라로 bowl 인식 → 3D 좌표 → 로봇 베이스 좌표 변환'),
    to_do('화면에 변환된 좌표를 출력만 (로봇 이동 X)'),
    to_do('실제 위치와 비교하여 오차 측정'),
    to_do('목표: 오차 10mm 이내'),
])

code_test_coord = """# 좌표 변환만 테스트 (로봇 이동 없이!)
T_cam2tcp = np.load('T_cam2tcp.npy')

while True:
    # ... 카메라 + YOLO 추론 ...
    cam_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], depth)

    # 현재 TCP 포즈
    ret = robot.Get_Current_Arm_State(retry=1)
    tcp_pose = ret[1]  # [x, y, z, rx, ry, rz]

    # 변환
    base_coord = camera_to_base(cam_3d, T_cam2tcp, tcp_pose)
    print(f"카메라 3D: {cam_3d}")
    print(f"로봇 베이스: {base_coord} mm")
    print(f"→ 이 좌표가 실제 bowl 위치와 맞는지 확인!")"""

append([code_block(code_test_coord)])

append([
    heading(2, '4-2. "보고 → 안전 이동"'),
    to_do('변환된 좌표의 10cm 위(Z+100)로 먼저 이동'),
    to_do('bowl 바로 위에 정확히 오는지 눈으로 확인'),
    to_do('여러 위치에서 반복 테스트'),
    to_do('오차가 크면 → 3단계 캘리브레이션 재수행'),
])

code_safe_move = """# 안전 이동 테스트 (bowl 위 10cm으로만 이동)
base_coord = camera_to_base(cam_3d, T_cam2tcp, tcp_pose)
x, y, z = base_coord

# bowl 위 100mm에서 정지 (잡지 않음!)
safe_pos = [x, y, z + 100, 3.14, 0, 0]
print(f"이동 목표: {safe_pos}")
input("Enter를 누르면 로봇이 이동합니다... (비상정지 준비!)")
robot.Movel_Cmd(safe_pos, v=10, r=0)"""

append([code_block(code_safe_move)])

append([
    heading(2, '4-3. "보고 → 잡기" (최종 테스트)'),
    to_do('접근 → 하강 → 그리퍼 잡기'),
    to_do('들어올리기 → 목표 위치 이동'),
    to_do('놓기'),
    to_do('여러 위치에서 성공률 확인'),
])

code_pick = """def pick_bowl(robot, base_coord):
    x, y, z = base_coord
    rx, ry, rz = 3.14, 0, 0  # 수직 하강 자세

    # 1. bowl 위 50mm 접근
    robot.Movel_Cmd([x, y, z+50, rx, ry, rz], v=20, r=0)
    time.sleep(0.5)

    # 2. 하강 → 잡기 위치
    robot.Movel_Cmd([x, y, z, rx, ry, rz], v=10, r=0)
    time.sleep(0.5)

    # 3. 그리퍼 잡기
    robot.Set_Gripper_Pick_On(500, 500)
    time.sleep(1.0)

    # 4. 들어올리기
    robot.Movel_Cmd([x, y, z+100, rx, ry, rz], v=15, r=0)
    time.sleep(0.5)

    # 5. 목표 위치로 이동
    PLACE_POS = [400, 0, 200, 3.14, 0, 0]  # 하드코딩 또는 설정
    robot.Movel_Cmd(PLACE_POS, v=30, r=0)
    time.sleep(0.5)

    # 6. 놓기
    robot.Set_Gripper_Release(500)
    time.sleep(0.5)

    print("Pick & Place 완료!")"""

append([
    code_block(code_pick),
    divider(),
])

# ══════════════════════════════════════════════════════════════
# 5단계: 자동화
# ══════════════════════════════════════════════════════════════
print('5단계: 자동화...')
append([
    heading(1, '5단계: 전체 자동화'),
    callout('4단계까지 성공하면, 전체를 하나의 루프로 통합', '🔄'),
])

append([
    heading(2, '5-1. 자동화 파이프라인'),
    to_do('전체 파이프라인을 하나의 루프로 통합'),
    to_do('예외처리: detection 실패 → 재촬영'),
    to_do('예외처리: depth 이상값 (0 또는 너무 먼 값) → 주변 median 보정'),
    to_do('예외처리: 그리퍼 실패 → 재시도'),
    to_do('예외처리: 관절 한계 → 다른 경로 시도 또는 스킵'),
    to_do('연속 동작: 여러 bowl 순차 처리'),
    to_do('속도 최적화: 이동 구간은 빠르게, 접근 구간은 천천히'),
])

code_auto = """# 전체 자동화 루프 (개념)
def auto_pick_place_loop():
    T_cam2tcp = np.load('T_cam2tcp.npy')

    while True:
        # 1. 촬영
        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)
        color = np.asanyarray(aligned.get_color_frame().get_data())
        depth_frame = aligned.get_depth_frame()
        intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

        # 2. 추론
        results = model(color, conf=0.7)
        bowls = extract_bowl_centers(results)  # [(cx, cy), ...]

        if not bowls:
            print("bowl 없음 → 대기")
            time.sleep(1)
            continue

        # 3. 각 bowl에 대해 pick & place
        for cx, cy in bowls:
            depth = get_stable_depth(depth_frame, cx, cy)  # median 보정
            if depth < 0.1 or depth > 1.0:
                continue

            cam_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [cx,cy], depth)
            tcp_pose = robot.Get_Current_Arm_State(retry=1)[1]
            base_coord = camera_to_base(cam_3d, T_cam2tcp, tcp_pose)

            pick_bowl(robot, base_coord)

        print("한 사이클 완료 → 다음 스캔")"""

append([code_block(code_auto)])

# ══════════════════════════════════════════════════════════════
# 진행 체크리스트 (요약)
# ══════════════════════════════════════════════════════════════
print('진행 체크리스트...')
append([
    divider(),
    heading(1, '📋 전체 진행 체크리스트'),
    callout('1단계 → 2단계까지 빠르게 확인 가능. 3단계(캘리브레이션)가 핵심이자 가장 시간 소요.', '⏱'),
])

append([
    heading(2, '1단계: 개별 장비 ✅'),
    to_do('로봇팔 연결 + 이동 테스트'),
    to_do('RealSense RGB + Depth 스트림'),
    to_do('YOLO Seg 실시간 추론'),
])

append([
    heading(2, '2단계: 3D 좌표'),
    to_do('Detection → depth → 카메라 3D 좌표 변환'),
    to_do('depth 안정성 확인 (반복 측정)'),
])

append([
    heading(2, '3단계: 캘리브레이션 (핵심)'),
    to_do('ArUco 마커 준비'),
    to_do('5~10개 자세에서 데이터 수집'),
    to_do('cv2.calibrateHandEye() 실행'),
    to_do('변환 정확도 검증 (오차 10mm 이내)'),
])

append([
    heading(2, '4단계: 통합 테스트'),
    to_do('좌표 출력만 (이동 X)'),
    to_do('안전 이동 (Z+100mm)'),
    to_do('실제 Pick & Place'),
])

append([
    heading(2, '5단계: 자동화'),
    to_do('루프 통합 + 예외처리'),
    to_do('연속 동작 + 속도 최적화'),
])

print('\n✅ 페이지 생성 완료!')
print(f'페이지 ID: {PAGE_ID}')
