"""
Notion - RealMan 로봇 제어 정리 페이지 (v2 - 통합 업데이트)
기존 페이지 삭제 후 재생성 (Q&A 내용 통합)
"""
import json, urllib.request, sys, time
sys.stdout.reconfigure(encoding='utf-8')

API_KEY = 'ntn_b57077336502qA3Deg7jq4qWxqnc8navEmCPcNhjyv7gn7'
RM_PAGE_ID = '32115435-1a92-81d0-816a-f016c4cecf14'  # 기존 RealMan 페이지

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

def append(children):
    """페이지에 블록 추가 (최대 100개씩)"""
    for i in range(0, len(children), 100):
        batch = children[i:i+100]
        api(f'https://api.notion.com/v1/blocks/{RM_PAGE_ID}/children', {
            'children': batch
        }, method='PATCH')
        time.sleep(0.3)

# ──────────────────────────────────────────────────────────────
# Step 0: 기존 블록 전부 삭제
# ──────────────────────────────────────────────────────────────
print('기존 블록 삭제 중...')
blocks = api(f'https://api.notion.com/v1/blocks/{RM_PAGE_ID}/children?page_size=100')
for block in blocks.get('results', []):
    try:
        api(f'https://api.notion.com/v1/blocks/{block["id"]}', method='DELETE')
    except:
        pass
# 2페이지 이상일 수 있으므로
while blocks.get('has_more'):
    blocks = api(f'https://api.notion.com/v1/blocks/{RM_PAGE_ID}/children?page_size=100&start_cursor={blocks["next_cursor"]}')
    for block in blocks.get('results', []):
        try:
            api(f'https://api.notion.com/v1/blocks/{block["id"]}', method='DELETE')
        except:
            pass
print('  삭제 완료')

# ══════════════════════════════════════════════════════════════
# Section 1: 프로젝트 전략
# ══════════════════════════════════════════════════════════════
print('섹션 1: 프로젝트 전략...')
append([
    callout('RealMan 로봇팔 제어 + 카메라 캘리브레이션 + 통합 파이프라인 정리', '🦾'),
    divider(),
    heading(1, '1. 프로젝트 전략: 1차 목표 vs 최종 목표'),
    paragraph(text('결론: ', bold=True), text('1차 목표부터 시작하되, 최종 목표를 고려한 설계', color='blue', bold=True)),
])

append([
    heading(2, '1-1. 목표 비교'),
    table(3, [
        ['구분', '1차 목표', '최종 목표'],
        ['배치', '일정 간격, 가지런한 배치', '무작위 흐트러진 배치'],
        ['인식', '고정 위치 기반 (좌표 하드코딩 가능)', 'Seg 모델 실시간 인식 필수'],
        ['난이도', '로봇 제어 기초 + 간단한 좌표 매핑', '고정밀 Seg + 동적 좌표 변환'],
        ['모델 정확도', '90%+ 충분', '99%+ 필요'],
        ['캘리브레이션', '기본 Eye-to-Hand', '정밀 캘리브레이션 + 보정'],
    ]),
])

append([
    heading(2, '1-2. 추천 전략: 단계적 접근'),
    callout('1차 목표를 먼저 완성하면서, 최종 목표에 재사용 가능한 구조로 설계', '💡'),
    heading(3, 'Phase 1: 고정 위치 Pick & Place (1차 목표)'),
    bullet('그릇 거치대 위치를 하드코딩 (로봇 좌표 직접 입력)'),
    bullet('카메라 없이도 동작 가능 → 로봇 제어 기초 확립'),
    bullet('핵심: movej/movel 명령어, 그리퍼 제어, 안전 로직'),
    heading(3, 'Phase 2: 카메라 기반 인식 추가'),
    bullet('RealSense 카메라 + Seg 모델로 그릇 위치 실시간 감지'),
    bullet('카메라-로봇 캘리브레이션 (Eye-to-Hand)'),
    bullet('감지된 좌표로 로봇 이동 → Phase 1의 하드코딩 좌표를 대체'),
    heading(3, 'Phase 3: 동적 배치 대응 (최종 목표)'),
    bullet('흐트러진 그릇 인식 → 개별 마스크에서 중심점/각도 추출'),
    bullet('충돌 회피, 순서 최적화 등 고급 로직 추가'),
    bullet('99%+ 인식률 필요'),
])

append([
    heading(2, '1-3. 왜 1차 목표부터 시작해야 하는가?'),
    numbered('로봇 제어 기초 확립: 좌표계, 속도, 그리퍼 등 기본기를 먼저 익힘'),
    numbered('디버깅 용이: 문제 발생 시 "로봇 문제 vs 인식 문제" 분리 가능'),
    numbered('빠른 성과: 1~2주 내 데모 가능 → 프로젝트 진행 증명'),
    numbered('코드 재사용: Phase 1의 로봇 제어 코드가 Phase 2~3에서 그대로 사용됨'),
    numbered('Seg 모델 학습과 병렬 진행: 로봇 제어 개발하는 동안 모델 정확도 향상'),
    divider(),
])

# ══════════════════════════════════════════════════════════════
# Section 2: RM65 하드웨어 & 컴퓨팅 구조 (NEW)
# ══════════════════════════════════════════════════════════════
print('섹션 2: 하드웨어 & 컴퓨팅 구조...')
append([
    heading(1, '2. RM65 하드웨어 & 컴퓨팅 구조'),
    heading(2, '2-1. RM65 사양'),
    table(2, [
        ['항목', '내용'],
        ['모델', 'RM65-B (6-DOF)'],
        ['무게', '7.2kg (컨트롤러 포함) / 6kg (본체만)'],
        ['가반하중', '5kg (피크 9kg)'],
        ['작업반경', '610mm'],
        ['전원', 'DC 24V, 최대 200W (배터리 구동 가능)'],
        ['컨트롤러', '로봇 베이스에 내장 (별도 제어 캐비닛 없음)'],
        ['통신', 'TCP/IP, RS485, I/O'],
        ['소프트웨어', 'ROS/ROS2 공식 지원'],
    ]),
])

append([
    heading(2, '2-2. 컴퓨팅 구조: 연산은 어디서 하는가?'),
    callout('로봇은 모터 제어만 담당. 딥러닝/카메라/좌표변환은 전부 외부 제어 PC에서 수행.', '⚡'),
])

code_computing = """# 컴퓨팅 구조도
#
# [외부 제어 PC]                          [RM65 로봇]
# ┌─────────────────────────┐           ┌──────────────┐
# │  RealSense 카메라 (USB)  │           │  내장 컨트롤러 │
# │  YOLOv8 Seg 추론 (GPU)  │  TCP/IP   │  모터 서보 제어│
# │  깊이 → 3D 변환          │ ────────→ │  모션 계획    │
# │  캘리브레이션 연산         │  이동 명령 │  궤적 실행    │
# │  rm_movel() 명령 전송    │           │              │
# └─────────────────────────┘           └──────────────┘
#
# 로봇은 "바보" → [300, 100, 150]으로 가라는 명령만 수행"""

append([
    code_block(code_computing, 'plain text'),
])

append([
    heading(2, '2-3. AMR + RM65 복합로봇 구조'),
    callout('AMR 내부에 메인 컴퓨터(Jetson/NUC/산업용PC)가 있고, 로봇팔과 TCP/IP로 통신', '🤖'),
])

code_amr = """# AMR + RM65 복합로봇 구조
#
# ┌──────────────────────────────────────────────┐
# │  AMR (자율 이동 로봇)                          │
# │                                              │
# │  ┌────────────────────────┐                  │
# │  │  메인 컴퓨터             │ ← Jetson/NUC/   │
# │  │  - ROS2 마스터           │    산업용 PC     │
# │  │  - 네비게이션            │                  │
# │  │  - YOLOv8 추론 ★        │                  │
# │  │  - 캘리브레이션 연산 ★    │                  │
# │  └──────────┬─────────────┘                  │
# │             │ TCP/IP (192.168.1.x)           │
# │  ┌──────────▼─────────────┐                  │
# │  │  RM65 로봇팔             │                  │
# │  │  (내장 컨트롤러)          │                  │
# │  └────────────────────────┘                  │
# │                                              │
# │  RealSense (USB → 메인 컴퓨터)                │
# │  배터리, LiDAR 등                             │
# └──────────────────────────────────────────────┘
#
# SSH로 접속하는 대상 = AMR 안의 메인 컴퓨터"""

append([
    code_block(code_amr, 'plain text'),
    bullet('RM65는 별도 PLC 불필요 (컨트롤러 내장)'),
    bullet('AMR과 로봇팔은 PLC가 아닌 TCP/IP 또는 ROS2 토픽으로 통신'),
    bullet('RealMan이 공식 ROS/ROS2 패키지 제공 → AMR의 ROS 시스템과 바로 통합'),
    bullet('SSH로 접속하는 대상 = AMR 내부 메인 컴퓨터 (여기서 모든 연산 수행)'),
    paragraph(text('확인 필요: ', bold=True), text('AMR 안에 어떤 컴퓨터가 있는지 (Jetson? NUC?) → GPU 유무에 따라 모델 추론 전략이 달라짐')),
    divider(),
])

# ══════════════════════════════════════════════════════════════
# Section 3: 카메라-로봇 캘리브레이션
# ══════════════════════════════════════════════════════════════
print('섹션 3: 캘리브레이션...')
append([
    heading(1, '3. 카메라-로봇 캘리브레이션 (Hand-Eye Calibration)'),
    callout('카메라가 보는 좌표 → 로봇이 이해하는 좌표로 변환하는 과정', '📷'),
])

# NEW: RealSense RGB vs Depth 설명
append([
    heading(2, '3-1. RealSense 카메라 구조: RGB vs Depth'),
    callout('딥러닝 학습은 RGB로, 3D 좌표 변환은 Depth로. 두 스트림은 동시에 사용 가능.', '💡'),
    paragraph(text('RealSense에는 물리적으로 카메라가 2개 내장:')),
    table(4, [
        ['센서', '출력', '용도', '딥러닝 관계'],
        ['RGB 카메라', '컬러 이미지 (1280x720)', 'YOLOv8 Seg 추론', 'RGB로 학습 → RGB로 추론 (정상)'],
        ['IR 스테레오', '깊이 맵 (depth)', '3D 좌표 변환', '모델과 무관, 좌표 변환에만 사용'],
    ]),
])

code_dual_stream = """# RGB + Depth 동시 사용 (실제 운용 코드)
import pyrealsense2 as rs

config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)  # RGB
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)   # Depth

pipeline = rs.pipeline()
pipeline.start(config)

# 프레임 획득 → RGB와 Depth 정렬
frames = pipeline.wait_for_frames()
aligned = rs.align(rs.stream.color).process(frames)

color_frame = aligned.get_color_frame()  # → YOLOv8에 입력
depth_frame = aligned.get_depth_frame()  # → 3D 좌표 변환에 사용

# 흐름:
# 1. color_frame → YOLOv8 → 그릇 마스크 → 중심점 (cx, cy) 픽셀
# 2. depth_frame.get_distance(cx, cy) → 깊이값 (미터)
# 3. (cx, cy, depth) → rs2_deproject → 카메라 3D 좌표"""

append([
    code_block(code_dual_stream),
    bullet('Depth 모드에서 보이는 "점" = IR 프로젝터의 적외선 패턴 (RGB에는 영향 없음)'),
    bullet('딥러닝 학습: RGB로 한 것이 정상. Depth는 학습에 사용하지 않음'),
    bullet('실제 운용: RGB(추론) + Depth(좌표) 동시에 켜서 사용'),
])

# 3가지 좌표계
append([
    heading(2, '3-2. 3가지 좌표계 이해'),
    table(4, [
        ['좌표계', '단위', '원점', '설명'],
        ['이미지 좌표 (u, v)', '픽셀 (px)', '이미지 좌상단', '카메라 센서가 찍은 2D 위치'],
        ['카메라 3D (Xc, Yc, Zc)', '미터 (m)', '카메라 렌즈 중심', 'RealSense 깊이로 복원한 실제 3D 위치'],
        ['로봇 좌표 (Xr, Yr, Zr)', '밀리미터 (mm)', '로봇 베이스', '로봇이 이동할 때 사용하는 좌표'],
    ]),
])

# 좌표 변환 흐름
append([
    heading(2, '3-3. 좌표 변환 흐름'),
    callout('이미지(u,v) → [RealSense 깊이] → 카메라 3D(Xc,Yc,Zc) → [캘리브레이션 행렬] → 로봇(Xr,Yr,Zr)', '🔄'),
    heading(3, 'Step 1: 이미지 → 카메라 3D'),
    paragraph(text('RealSense의 '), text('rs2_deproject_pixel_to_point()', code=True), text(' 함수 사용')),
    bullet('입력: 픽셀 좌표 (u, v) + 해당 픽셀의 깊이값 (depth)'),
    bullet('출력: 카메라 기준 3D 좌표 (Xc, Yc, Zc) [미터]'),
    bullet('카메라 내부 파라미터(intrinsics)를 자동으로 사용'),
])

code_step1 = """import pyrealsense2 as rs
import numpy as np

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)

frames = pipeline.wait_for_frames()
aligned = rs.align(rs.stream.color).process(frames)
depth_frame = aligned.get_depth_frame()
intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

# 픽셀 (u, v) → 카메라 3D 좌표
u, v = 640, 360  # 예시: 이미지 중앙
depth = depth_frame.get_distance(u, v)  # 미터 단위
point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [u, v], depth)
# point_3d = [Xc, Yc, Zc] (미터)"""

append([
    code_block(code_step1),
])

# Step 2: SVD 캘리브레이션
append([
    heading(3, 'Step 2: 카메라 3D → 로봇 좌표 (캘리브레이션)'),
    paragraph(text('핵심: ', bold=True), text('같은 점을 카메라와 로봇 양쪽에서 측정한 뒤, 변환 행렬(R, t)을 계산')),
    bullet('R: 3x3 회전 행렬 (카메라와 로봇의 축 방향 차이)'),
    bullet('t: 3x1 이동 벡터 (카메라와 로봇 원점의 거리 차이)'),
    bullet('변환 공식: P_robot = R × P_camera + t'),
    heading(3, '캘리브레이션 데이터 수집 방법'),
    numbered('로봇 팔 끝에 마커(펜 끝 등)를 부착'),
    numbered('작업 영역 내 5~10개 지점으로 로봇을 이동'),
    numbered('각 지점에서: 로봇 좌표 기록 + 카메라로 마커 위치 클릭하여 3D 좌표 획득'),
    numbered('5~10쌍의 (카메라 좌표, 로봇 좌표)로 SVD 계산'),
])

code_svd = """import numpy as np

def calibrate(camera_points, robot_points):
    \"\"\"
    카메라 3D 좌표 → 로봇 좌표 변환 행렬 계산 (SVD 방식)
    camera_points: np.array (N, 3) - 카메라 좌표 [미터]
    robot_points:  np.array (N, 3) - 로봇 좌표 [밀리미터]
    return: 4x4 변환 행렬 T
    \"\"\"
    cam_center = camera_points.mean(axis=0)
    rob_center = robot_points.mean(axis=0)
    cam_centered = camera_points - cam_center
    rob_centered = robot_points - rob_center

    H = cam_centered.T @ rob_centered
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:  # 반사 보정
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = rob_center - R @ cam_center
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

def camera_to_robot(T, camera_point):
    p = np.append(camera_point, 1)
    return (T @ p)[:3]"""

append([code_block(code_svd)])

code_collect = """# 캘리브레이션 포인트 수집 예시
camera_points = np.array([
    [0.12, -0.05, 0.85],   # 점 1 (미터)
    [-0.10, 0.08, 0.82],   # 점 2
    [0.05, 0.15, 0.88],    # ...최소 5개, 권장 8~10개
])
robot_points = np.array([
    [350.2, -120.5, 150.3],   # 같은 점 1 (밀리미터)
    [180.1, 95.8, 165.2],     # 같은 점 2
    [280.5, 200.3, 130.1],    # ...
])

T = calibrate(camera_points, robot_points)
np.save('calibration_matrix.npy', T)  # 저장해두고 재사용

# 테스트
test_cam = np.array([0.08, 0.03, 0.86])
test_rob = camera_to_robot(T, test_cam)
print(f"카메라: {test_cam} → 로봇: {test_rob}")"""

append([code_block(code_collect)])

# Eye-to-Hand
append([
    heading(2, '3-4. Eye-to-Hand vs Eye-in-Hand'),
    table(3, [
        ['구분', 'Eye-to-Hand (현재 설정)', 'Eye-in-Hand'],
        ['카메라 위치', '로봇 머리(고정 위치)에 장착', '로봇 팔 끝에 장착'],
        ['장점', '캘리브레이션 1회만 하면 됨', '팔이 가까이 가서 정밀도 높음'],
        ['단점', '거리에 따라 정밀도 저하', '팔 움직일 때마다 좌표 재계산'],
        ['적합 상황', '넓은 영역 감시, 고정 작업', '정밀 조립, 좁은 영역'],
    ]),
    callout('현재 프로젝트는 카메라가 로봇 머리에 고정 → Eye-to-Hand 방식. 캘리브레이션을 한 번만 하면 계속 사용 가능.', '✅'),
])

# 캘리브레이션 팁
append([
    heading(2, '3-5. 캘리브레이션 실전 팁'),
    numbered('포인트 분포: 작업 영역 전체에 골고루 (한쪽에 몰리면 정밀도 저하)'),
    numbered('높이 다양성: 같은 높이만 측정하지 말고, 다른 높이의 점도 포함'),
    numbered('단위 주의: 카메라=미터(m), 로봇=밀리미터(mm)'),
    numbered('검증: 캘리브레이션 후 테스트 포인트로 오차 확인 (목표: 5mm 이내)'),
    numbered('재캘리브레이션: 카메라 위치가 바뀌면 반드시 다시 수행'),
    divider(),
])

# ══════════════════════════════════════════════════════════════
# Section 4: RealMan SDK
# ══════════════════════════════════════════════════════════════
print('섹션 4: RealMan SDK...')
append([
    heading(1, '4. RealMan SDK 정리'),
    heading(2, '4-1. 설치 및 연결'),
    table(2, [
        ['항목', '내용'],
        ['설치', 'pip install Robotic_Arm'],
        ['Python 버전', '3.9+'],
        ['연결 IP', '192.168.1.18'],
        ['포트', '8080'],
        ['통신', 'TCP/IP'],
        ['GitHub', 'https://github.com/RealManRobot/RM_API2'],
        ['공식 문서', 'https://develop.realman-robotics.com/en/robot/apipython/getStarted/'],
    ]),
])

code_connect = """from Robotic_Arm.rm_robot_interface import *

robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
handle = robot.rm_create_robot_arm("192.168.1.18", 8080)

# 소프트웨어 정보 조회
info = robot.rm_get_arm_software_info()
print(f"로봇 정보: {info}")

# 현재 로봇 상태 (관절각 + TCP 위치)
state = robot.rm_get_current_arm_state()
# state → 관절각: [j1~j6] (도), TCP: [x,y,z,rx,ry,rz] (mm, 도)

robot.rm_delete_robot_arm()  # 연결 해제"""

append([code_block(code_connect)])

# 이동 명령어
append([
    heading(2, '4-2. 이동 명령어'),
    table(4, [
        ['명령어', '타입', '용도', '특징'],
        ['rm_movej', '관절 공간', '빠른 이동, 대기 위치', '최단 경로(관절 기준), 경로 예측 어려움'],
        ['rm_movel', '직선 이동', '정밀 작업, 접근/후퇴', 'TCP가 직선으로 이동, 속도 느림'],
        ['rm_movec', '원호 이동', '곡선 경로 필요 시', '3점(시작/중간/끝)으로 원호 정의'],
        ['rm_movej_p', '관절→포즈', '목표 포즈로 빠른 이동', '관절 공간이지만 목표는 직교 좌표'],
    ]),
])

code_move = """# 좌표: [x, y, z, rx, ry, rz] (mm, rad)

# 1. 직선 이동 (정밀 작업) - 가장 많이 사용
robot.rm_movel([300.0, 0.0, 200.0, 3.14, 0.0, 0.0], 20)

# 2. 관절 이동 (빠른 이동)
robot.rm_movej([0.0, -30.0, 60.0, 0.0, 90.0, 0.0], 30)

# 3. 관절 이동으로 포즈 도달
robot.rm_movej_p([300.0, 0.0, 200.0, 3.14, 0.0, 0.0], 30)

# 4. 원호 이동 (중간점 + 끝점)
robot.rm_movec(
    [310, 50, 200, 3.14, 0, 0],   # 중간점 (via)
    [300, 100, 200, 3.14, 0, 0],  # 끝점 (end)
    speed=15
)"""

append([code_block(code_move)])

# 그리퍼
append([
    heading(2, '4-3. 그리퍼(Gripper) 제어'),
])

code_gripper = """# 그리퍼 (로봇 모델/그리퍼 타입에 따라 API 다를 수 있음)
robot.rm_set_gripper_release(500)        # 열기 (놓기)
robot.rm_set_gripper_pick(500, 200)      # 닫기 (잡기) - 속도, 힘

# IO 기반 그리퍼 (일부 모델)
robot.rm_set_tool_DO(1, True)   # ON
robot.rm_set_tool_DO(1, False)  # OFF"""

append([
    code_block(code_gripper),
    divider(),
])

# ══════════════════════════════════════════════════════════════
# Section 5: 통합 파이프라인
# ══════════════════════════════════════════════════════════════
print('섹션 5: 통합 파이프라인...')
append([
    heading(1, '5. 전체 통합 파이프라인'),
    callout('Seg 모델 → 카메라 3D → 로봇 좌표 → 이동 명령', '🔗'),
    heading(2, '5-1. 파이프라인 흐름도'),
    numbered('RealSense 컬러+깊이 프레임 획득 (동시)'),
    numbered('YOLOv8 Seg 모델로 그릇 마스크 감지 (RGB 사용, conf 0.7+)'),
    numbered('마스크 중심점 (cx, cy) 계산 → 픽셀 좌표'),
    numbered('Depth 프레임에서 (cx, cy) → 카메라 3D 좌표 변환'),
    numbered('캘리브레이션 행렬로 카메라 3D → 로봇 좌표 변환'),
    numbered('로봇에 movel 명령 전송 → 그릇 위치로 이동'),
    numbered('그리퍼로 그릇 잡기 → 목표 위치로 이동 → 놓기'),
])

code_pipeline_1 = """import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
from Robotic_Arm.rm_robot_interface import *

# ─── 초기화 ─────────────────────────────────
# 1. RealSense (RGB + Depth 동시)
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
pipeline.start(config)
align = rs.align(rs.stream.color)

# 2. YOLOv8 Seg 모델
model = YOLO('runs/segment/bowl_seg_v3/weights/best.pt')

# 3. 로봇 연결
robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
robot.rm_create_robot_arm("192.168.1.18", 8080)

# 4. 캘리브레이션 행렬
T = np.load('calibration_matrix.npy')"""

code_pipeline_2 = """def detect_and_pick():
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    color_image = np.asanyarray(color_frame.get_data())
    intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

    # Seg 추론 (RGB만 사용)
    results = model(color_image, conf=0.7)

    for r in results:
        if r.masks is None:
            continue
        for mask in r.masks.xy:
            cx = int(mask[:, 0].mean())
            cy = int(mask[:, 1].mean())

            # Depth에서 깊이값 읽기
            depth = depth_frame.get_distance(cx, cy)
            if depth < 0.1 or depth > 1.5:
                continue

            # 픽셀 → 카메라 3D → 로봇 좌표
            cam_3d = rs.rs2_deproject_pixel_to_point(
                intrinsics, [cx, cy], depth)
            rob_point = T[:3,:3] @ np.array(cam_3d) + T[:3,3]
            x, y, z = rob_point

            # 로봇 이동 (Pick & Place)
            robot.rm_movel([x, y, z+50, 3.14, 0, 0], 30)
            robot.rm_movel([x, y, z, 3.14, 0, 0], 10)
            robot.rm_set_gripper_pick(500, 200)
            robot.rm_movel([x, y, z+100, 3.14, 0, 0], 20)
            robot.rm_movel([400, 0, 200, 3.14, 0, 0], 30)
            robot.rm_set_gripper_release(500)
            return True
    return False"""

append([
    heading(2, '5-2. 통합 코드 예시'),
    code_block(code_pipeline_1),
    code_block(code_pipeline_2),
    divider(),
])

# ══════════════════════════════════════════════════════════════
# Section 6: Phase 1 구현 가이드 (확장)
# ══════════════════════════════════════════════════════════════
print('섹션 6: Phase 1 구현 가이드...')
append([
    heading(1, '6. Phase 1 구현 가이드 (고정 위치 Pick & Place)'),
    callout('카메라/Seg 모델 없이, 하드코딩된 좌표로 로봇 제어 먼저 완성', '🎯'),
])

# NEW: 그릇 옮기기 변수 사항
append([
    heading(2, '6-1. 그릇 옮길 때 고려할 변수'),
    heading(3, '① 그리퍼 관련'),
    table(2, [
        ['변수', '설명'],
        ['그릇 크기/형태', '지름, 높이, 테두리 두께가 다를 수 있음'],
        ['잡는 위치', '위에서? 옆에서? 그릇 형태에 따라 전략 다름'],
        ['잡는 힘', '너무 세면 파손, 너무 약하면 미끄러짐'],
        ['그리퍼 타입', '2핑거 / 흡착 / 커스텀에 따라 접근 방식 다름'],
    ]),
])

append([
    heading(3, '② 접근 경로 & 충돌'),
    bullet('옆 그릇과의 간격: 그리퍼가 들어갈 공간이 충분한지'),
    bullet('거치대 높이: 들어올릴 때 거치대에 걸리지 않는지'),
    bullet('접근 각도: 수직 하강만 가능한지, 비스듬히 들어가야 하는지'),
    bullet('그릇 사이 간격 좁으면: 한 그릇 잡을 때 옆 그릇 건드릴 수 있음'),
])

append([
    heading(3, '③ 높이(Z축) 변수'),
    table(2, [
        ['상황', '문제'],
        ['그릇이 쌓여있는 경우', '1개 집을 때마다 높이가 달라짐 (스택)'],
        ['거치대 높이 오차', '거치대 자체가 정확히 수평이 아닐 수 있음'],
        ['그릇 바닥 형태', '평평/볼록에 따라 안착 높이 다름'],
        ['컨베이어 높이', '움직이면서 미세하게 높이 변할 수 있음'],
    ]),
])

append([
    heading(3, '④ 놓기(Place) 관련'),
    bullet('놓는 위치 정밀도: 정확한 위치에 안착하는지'),
    bullet('그릇끼리 겹침: 이미 놓은 그릇과 부딪히지 않는지'),
    bullet('놓는 순서: 여러 그릇 옮길 때 효율적인 순서'),
    bullet('충격: 급하게 내려놓으면 그릇 파손 또는 음식 흘러넘침'),
])

append([
    heading(3, '⑤ 속도 & 안전'),
    bullet('구간별 속도 조절: 이동 구간은 빠르게, 접근 구간은 천천히'),
    bullet('가감속: 급정거하면 잡고 있는 그릇이 흔들림'),
    bullet('음식 담긴 경우: 기울어지면 안 됨 → 자세(rx,ry,rz) 수평 유지 필수'),
])

append([
    heading(3, '실전에서 자주 겪는 문제 Top 5'),
    numbered('그리퍼가 그릇을 못 잡음 (위치 약간 틀림, 그릇 젖어있음)'),
    numbered('옆 그릇 건드림 (간격 부족)'),
    numbered('놓을 때 그릇이 기울어짐 (놓는 높이 부정확)'),
    numbered('이동 중 그릇 미끄러짐 (속도 너무 빠름, 잡는 힘 부족)'),
    numbered('관절 한계 도달 (특정 위치로 갈 수 없는 자세)'),
])

# NEW: 갈고리형 거치대 접근 전략
append([
    heading(2, '6-2. 갈고리형 거치대 접근 전략'),
    callout('거치대 밑이 갈고리 형태 → 수직(Z축만)으로는 빼낼 수 없음. 기울여서 빼는 웨이포인트 경로 필요.', '⚠️'),
])

code_hook_problem = """# 문제 상황: 수직 하강으로는 갈고리에 걸림
#
#          그리퍼
#            ↓
#      ┌───┐   ┌───┐  ← 갈고리 (걸림)
#      │   │ ○ │   │  ← 그릇
#      │   └───┘   │
#      │  거치대    │
#      └───────────┘
#
# 해결: 기울여서 빼는 경로 (웨이포인트)
#
#     ④ 상승 (자유 공간, 수평 복원)
#     ↑
#     ③ 기울여서 빼기 (갈고리 클리어)
#    ╱
#   ② 잡은 상태에서 살짝 기울임
#   │
#   ① 접근 → 잡기"""

append([code_block(code_hook_problem, 'plain text')])

append([
    heading(3, '방법 1: 웨이포인트 수동 티칭 (가장 현실적)'),
    paragraph(text('로봇을 교시 모드(free-drive)로 전환 → 사람이 직접 손으로 경로 탐색 → 좌표 기록')),
])

code_waypoint = """# 교시 모드로 기록한 웨이포인트 (실제 좌표는 티칭으로 얻어야 함)
PICK_SEQUENCE = [
    # [x, y, z, rx, ry, rz], speed, description
    ([300, -100, 200, 3.14, 0, 0],     30, "그릇 위 접근"),
    ([300, -100, 155, 3.14, 0, 0],     10, "하강 → 잡기 위치"),
    # --- 그리퍼 잡기 ---
    ([300, -100, 155, 3.14, 0.15, 0],   5, "살짝 기울임 (ry 변경)"),
    ([300, -100, 165, 3.14, 0.25, 0],   5, "기울이며 위로 (갈고리 회피)"),
    ([300, -100, 180, 3.14, 0.15, 0],  10, "더 올리며 각도 복원"),
    ([300, -100, 220, 3.14, 0, 0],     20, "자유 공간 → 수평 복원"),
]

def pick_from_rack(robot):
    robot.rm_movel(PICK_SEQUENCE[0][0], PICK_SEQUENCE[0][1])
    time.sleep(0.5)
    robot.rm_movel(PICK_SEQUENCE[1][0], PICK_SEQUENCE[1][1])
    time.sleep(0.5)
    robot.rm_set_gripper_pick(500, 200)  # 잡기
    time.sleep(1.0)
    # 기울여서 빼기 (핵심!)
    for waypoint, speed, desc in PICK_SEQUENCE[2:]:
        print(f"  {desc}")
        robot.rm_movel(waypoint, speed)
        time.sleep(0.3)"""

append([code_block(code_waypoint)])

append([
    heading(3, '방법 2: 원호 이동 (movec)'),
    paragraph(text('갈고리를 피하는 경로가 곡선이면 movec가 자연스러움:')),
])

code_movec = """# 잡은 후 원호로 빼기
start = [300, -100, 155, 3.14, 0, 0]      # 잡은 상태
via   = [300, -100, 170, 3.14, 0.2, 0]    # 중간 (기울어진 상태)
end   = [300, -100, 210, 3.14, 0, 0]      # 끝 (갈고리 위, 수평)

robot.rm_movec(via, end, speed=8)"""

append([code_block(code_movec)])

append([
    heading(3, '방법 3: 관절 직접 제어 (4~6축 회전)'),
])

code_joint = """# 특정 관절(5축 = 손목 기울기)만 회전시켜서 빼기
state = robot.rm_get_current_arm_state()
current_joints = state['joint']  # [j1, j2, j3, j4, j5, j6]

# j5만 10도 기울임
tilt_joints = current_joints.copy()
tilt_joints[4] += 10  # j5 회전
robot.rm_movej(tilt_joints, 5)  # 저속
time.sleep(0.5)

# 기울인 상태에서 위로 들기
lift_joints = tilt_joints.copy()
lift_joints[1] -= 5   # j2 조정 (위로)
robot.rm_movej(lift_joints, 10)"""

append([code_block(code_joint)])

# 교시 좌표 기록 도우미
append([
    heading(3, '교시 좌표 기록 도우미 스크립트'),
])

code_teach = """from Robotic_Arm.rm_robot_interface import *
import json

robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
robot.rm_create_robot_arm("192.168.1.18", 8080)

waypoints = []
print("교시 모드 ON → 로봇을 원하는 위치로 이동")
print("저장: Enter / 종료: q + Enter")

while True:
    cmd = input(f"\\n포인트 {len(waypoints)+1} 저장 (Enter/q): ")
    if cmd.lower() == 'q':
        break
    state = robot.rm_get_current_arm_state()
    waypoints.append({
        'index': len(waypoints) + 1,
        'joint': state['joint'],
        'pose': state['pose'],
    })
    print(f"  관절: {state['joint']}")
    print(f"  TCP:  {state['pose']}")

with open('pick_waypoints.json', 'w') as f:
    json.dump(waypoints, f, indent=2)
print(f"총 {len(waypoints)}개 저장 → pick_waypoints.json")
robot.rm_delete_robot_arm()"""

append([code_block(code_teach)])

append([
    heading(3, '갈고리 거치대: 추천 테스트 순서'),
    numbered('교시 모드로 "빼는 경로" 탐색 (사람이 직접)'),
    numbered('5~8개 웨이포인트 기록'),
    numbered('movel로 순차 재생 (저속)'),
    numbered('미세 조정 반복'),
    numbered('안정되면 속도 점진적 증가'),
    callout('핵심은 rx, ry, rz (자세/기울기)를 바꾸면서 빼는 것. 단순 xyz 이동만으로는 갈고리를 피할 수 없음.', '💡'),
])

# Phase 1 기본 코드
append([
    heading(2, '6-3. Phase 1 기본 코드'),
])

code_phase1 = """from Robotic_Arm.rm_robot_interface import *
import time

robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
robot.rm_create_robot_arm("192.168.1.18", 8080)

HOME = [0.0, -30.0, 60.0, 0.0, 90.0, 0.0]  # 홈 위치 (관절각)
APPROACH_HEIGHT = 50  # mm

BOWL_POSITIONS = [  # 티칭으로 기록한 좌표
    [300.0, -100.0, 150.0, 3.14, 0.0, 0.0],
    [300.0,    0.0, 150.0, 3.14, 0.0, 0.0],
    [300.0,  100.0, 150.0, 3.14, 0.0, 0.0],
]
PLACE_POSITION = [400.0, 200.0, 150.0, 3.14, 0.0, 0.0]

def pick(pos):
    x, y, z, rx, ry, rz = pos
    robot.rm_movel([x, y, z+APPROACH_HEIGHT, rx, ry, rz], 30)
    time.sleep(0.5)
    robot.rm_movel(pos, 10)
    time.sleep(0.5)
    robot.rm_set_gripper_pick(500, 200)
    time.sleep(1.0)
    robot.rm_movel([x, y, z+APPROACH_HEIGHT, rx, ry, rz], 20)

def place(pos):
    x, y, z, rx, ry, rz = pos
    robot.rm_movel([x, y, z+APPROACH_HEIGHT, rx, ry, rz], 30)
    time.sleep(0.5)
    robot.rm_movel(pos, 10)
    time.sleep(0.5)
    robot.rm_set_gripper_release(500)
    time.sleep(1.0)
    robot.rm_movel([x, y, z+APPROACH_HEIGHT, rx, ry, rz], 20)

robot.rm_movej(HOME, 30)
for i, bowl_pos in enumerate(BOWL_POSITIONS):
    print(f"그릇 {i+1} 이동 중...")
    pick(bowl_pos)
    place(PLACE_POSITION)
    robot.rm_movej(HOME, 30)

print("완료!")
robot.rm_delete_robot_arm()"""

append([code_block(code_phase1)])

append([
    heading(2, '6-4. Phase 1 테스트 순서'),
    numbered('로봇 연결 테스트: ping → SDK 연결 → 상태 조회'),
    numbered('그리퍼 테스트: 그릇 하나만 놓고 잡기/놓기 반복 (힘, 위치 조정)'),
    numbered('단일 Pick & Place: 한 위치에서 잡아서 다른 위치에 놓기'),
    numbered('갈고리 회피 테스트: 교시로 빼는 경로 탐색'),
    numbered('충돌 확인: 여러 그릇 놓고 하나만 집을 때 옆 그릇 안 건드리는지'),
    numbered('연속 동작: 여러 그릇 순차 이동'),
    numbered('속도 올리기: 안정성 확인 후 점진적으로 증가'),
    divider(),
])

# ══════════════════════════════════════════════════════════════
# Section 7: 참고 & 주의
# ══════════════════════════════════════════════════════════════
print('섹션 7: 참고 & 주의사항...')
append([
    heading(1, '7. 참고 링크 및 리소스'),
    bullet('RealMan Python SDK: https://develop.realman-robotics.com/en/robot/apipython/getStarted/'),
    bullet('RealMan Demo: https://develop.realman-robotics.com/en/robot/demo/python/simpleProcess/'),
    bullet('RealMan GitHub: https://github.com/RealManRobot/RM_API2'),
    bullet('RealSense SDK: https://github.com/IntelRealSense/librealsense'),
    bullet('YOLOv8 문서: https://docs.ultralytics.com/'),
    divider(),
    heading(1, '8. 주의사항'),
    callout('로봇 동작 테스트 시 반드시 비상정지 버튼을 손에 쥐고 진행!', '⚠️'),
    numbered('처음 테스트는 반드시 저속(10% 이하)으로 진행'),
    numbered('작업 영역 내 사람이 없는지 확인'),
    numbered('관절 한계(joint limit) 초과하지 않도록 좌표 범위 확인'),
    numbered('그리퍼 힘 조절: 그릇이 깨지지 않을 정도로 (도자기 주의)'),
    numbered('캘리브레이션 후 검증 포인트로 오차 반드시 확인'),
])

print('\n완료!')
print('RealMan 로봇 제어 (v2 통합)')
print('  ├── 1. 프로젝트 전략 (1차 vs 최종 목표)')
print('  ├── 2. RM65 하드웨어 & 컴퓨팅 구조 ★NEW')
print('  │    ├── RM65 사양')
print('  │    ├── 컴퓨팅 구조 (어디서 연산하는가)')
print('  │    └── AMR + RM65 복합로봇 구조')
print('  ├── 3. 카메라-로봇 캘리브레이션')
print('  │    ├── RealSense RGB vs Depth 설명 ★NEW')
print('  │    ├── 좌표계 / 변환 흐름 / SVD')
print('  │    └── Eye-to-Hand / 실전 팁')
print('  ├── 4. RealMan SDK 정리')
print('  ├── 5. 전체 통합 파이프라인')
print('  ├── 6. Phase 1 구현 가이드')
print('  │    ├── 그릇 옮기기 변수 사항 ★NEW')
print('  │    ├── 갈고리형 거치대 접근 전략 ★NEW')
print('  │    ├── 교시 좌표 기록 도우미 ★NEW')
print('  │    └── 테스트 순서')
print('  ├── 7. 참고 링크')
print('  └── 8. 주의사항')
