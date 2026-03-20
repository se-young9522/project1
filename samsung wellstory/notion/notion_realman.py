"""
Notion - RealMan 로봇 제어 정리 페이지 생성
프로젝트 진행 일지 > RealMan 서브페이지
"""
import json, urllib.request, sys
sys.stdout.reconfigure(encoding='utf-8')

API_KEY = 'ntn_b57077336502qA3Deg7jq4qWxqnc8navEmCPcNhjyv7gn7'
PROJECT_PAGE_ID = '32015435-1a92-813d-a3f0-e433e57bc50a'

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
    # Notion 코드 블록 2000자 제한
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
    """cells: list of strings"""
    return {'object': 'block', 'type': 'table_row', 'table_row': {
        'cells': [[text(c)] for c in cells]
    }}

def table(width, rows):
    """rows: list of list of strings"""
    return {'object': 'block', 'type': 'table', 'table': {
        'table_width': width,
        'has_column_header': True,
        'has_row_header': False,
        'children': [table_row(r) for r in rows],
    }}

# ──────────────────────────────────────────────────────────────
# Step 1: RealMan 서브페이지 생성
# ──────────────────────────────────────────────────────────────
print('RealMan 서브페이지 생성...')
rm_page = api('https://api.notion.com/v1/pages', {
    'parent': {'page_id': PROJECT_PAGE_ID},
    'icon': {'type': 'emoji', 'emoji': '🦾'},
    'properties': {
        'title': {'title': [{'text': {'content': 'RealMan 로봇 제어'}}]}
    },
}, method='POST')
rm_page_id = rm_page['id']
print(f'  페이지 ID: {rm_page_id}')

# ──────────────────────────────────────────────────────────────
# Section 1: 프로젝트 전략 (1차 vs 최종 목표)
# ──────────────────────────────────────────────────────────────
print('섹션 1: 프로젝트 전략...')
api(f'https://api.notion.com/v1/blocks/{rm_page_id}/children', {
    'children': [
        callout('RealMan 로봇팔 제어 + 카메라 캘리브레이션 + 통합 파이프라인 정리', '🦾'),
        divider(),
        heading(1, '1. 프로젝트 전략: 1차 목표 vs 최종 목표'),
        paragraph(text('결론: ', bold=True), text('1차 목표부터 시작하되, 최종 목표를 고려한 설계', color='blue', bold=True)),
        divider(),
    ]
}, method='PATCH')

# 1차/최종 목표 비교 테이블
api(f'https://api.notion.com/v1/blocks/{rm_page_id}/children', {
    'children': [
        heading(2, '1-1. 목표 비교'),
        table(3, [
            ['구분', '1차 목표', '최종 목표'],
            ['배치', '일정 간격, 가지런한 배치', '무작위 흐트러진 배치'],
            ['인식', '고정 위치 기반 (좌표 하드코딩 가능)', 'Seg 모델 실시간 인식 필수'],
            ['난이도', '로봇 제어 기초 + 간단한 좌표 매핑', '고정밀 Seg + 동적 좌표 변환'],
            ['모델 정확도', '90%+ 충분', '99%+ 필요'],
            ['캘리브레이션', '기본 Eye-to-Hand', '정밀 캘리브레이션 + 보정'],
        ]),
    ]
}, method='PATCH')

# 추천 전략
api(f'https://api.notion.com/v1/blocks/{rm_page_id}/children', {
    'children': [
        heading(2, '1-2. 추천 전략: 단계적 접근'),
        callout('1차 목표를 먼저 완성하면서, 최종 목표에 재사용 가능한 구조로 설계', '💡'),
        heading(3, 'Phase 1: 고정 위치 Pick & Place (1차 목표)'),
        bullet('그릇 거치대 위치를 하드코딩 (로봇 좌표 직접 입력)'),
        bullet('카메라 없이도 동작 가능 → 로봇 제어 기초 확립'),
        bullet('핵심: movej/movel 명령어, 그리퍼 제어, 안전 로직'),
        bullet('예상 기간: 1~2주'),
        heading(3, 'Phase 2: 카메라 기반 인식 추가'),
        bullet('RealSense 카메라 + Seg 모델로 그릇 위치 실시간 감지'),
        bullet('카메라-로봇 캘리브레이션 (Eye-to-Hand)'),
        bullet('감지된 좌표로 로봇 이동 → Phase 1의 하드코딩 좌표를 대체'),
        bullet('예상 조건: Seg 모델 95%+ 달성 후'),
        heading(3, 'Phase 3: 동적 배치 대응 (최종 목표)'),
        bullet('흐트러진 그릇 인식 → 개별 마스크에서 중심점/각도 추출'),
        bullet('충돌 회피, 순서 최적화 등 고급 로직 추가'),
        bullet('99%+ 인식률 필요'),
        divider(),
    ]
}, method='PATCH')

# 왜 1차 목표부터?
api(f'https://api.notion.com/v1/blocks/{rm_page_id}/children', {
    'children': [
        heading(2, '1-3. 왜 1차 목표부터 시작해야 하는가?'),
        numbered('로봇 제어 기초 확립: 좌표계, 속도, 그리퍼 등 기본기를 먼저 익힘'),
        numbered('디버깅 용이: 문제 발생 시 "로봇 문제 vs 인식 문제" 분리 가능'),
        numbered('빠른 성과: 1~2주 내 데모 가능 → 프로젝트 진행 증명'),
        numbered('코드 재사용: Phase 1의 로봇 제어 코드가 Phase 2~3에서 그대로 사용됨'),
        numbered('Seg 모델 학습과 병렬 진행: 로봇 제어 개발하는 동안 모델 정확도 향상'),
        divider(),
    ]
}, method='PATCH')

# ──────────────────────────────────────────────────────────────
# Section 2: 카메라-로봇 캘리브레이션
# ──────────────────────────────────────────────────────────────
print('섹션 2: 캘리브레이션...')
api(f'https://api.notion.com/v1/blocks/{rm_page_id}/children', {
    'children': [
        heading(1, '2. 카메라-로봇 캘리브레이션 (Hand-Eye Calibration)'),
        callout('카메라가 보는 좌표 → 로봇이 이해하는 좌표로 변환하는 과정', '📷'),
        heading(2, '2-1. 3가지 좌표계 이해'),
    ]
}, method='PATCH')

# 좌표계 테이블
api(f'https://api.notion.com/v1/blocks/{rm_page_id}/children', {
    'children': [
        table(4, [
            ['좌표계', '단위', '원점', '설명'],
            ['이미지 좌표 (u, v)', '픽셀 (px)', '이미지 좌상단', '카메라 센서가 찍은 2D 위치'],
            ['카메라 3D 좌표 (Xc, Yc, Zc)', '미터 (m)', '카메라 렌즈 중심', 'RealSense 깊이로 복원한 실제 3D 위치'],
            ['로봇 좌표 (Xr, Yr, Zr)', '밀리미터 (mm)', '로봇 베이스', '로봇이 이동할 때 사용하는 좌표'],
        ]),
    ]
}, method='PATCH')

# 좌표 변환 흐름
api(f'https://api.notion.com/v1/blocks/{rm_page_id}/children', {
    'children': [
        heading(2, '2-2. 좌표 변환 흐름'),
        paragraph(text('전체 파이프라인:', bold=True)),
        callout('이미지(u,v) → [RealSense 깊이] → 카메라 3D(Xc,Yc,Zc) → [캘리브레이션 행렬] → 로봇(Xr,Yr,Zr)', '🔄'),
        heading(3, 'Step 1: 이미지 → 카메라 3D'),
        paragraph(text('RealSense의 '), text('rs2_deproject_pixel_to_point()', code=True), text(' 함수 사용')),
        bullet('입력: 픽셀 좌표 (u, v) + 해당 픽셀의 깊이값 (depth)'),
        bullet('출력: 카메라 기준 3D 좌표 (Xc, Yc, Zc) [미터]'),
        bullet('카메라 내부 파라미터(intrinsics)를 자동으로 사용'),
    ]
}, method='PATCH')

# Step 1 코드
code_step1 = """import pyrealsense2 as rs
import numpy as np

# RealSense 파이프라인 설정
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)

# 프레임 획득
frames = pipeline.wait_for_frames()
aligned = rs.align(rs.stream.color).process(frames)
depth_frame = aligned.get_depth_frame()
color_frame = aligned.get_color_frame()

# 카메라 내부 파라미터
intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

# 픽셀 (u, v) → 카메라 3D 좌표
u, v = 640, 360  # 예시: 이미지 중앙
depth = depth_frame.get_distance(u, v)  # 미터 단위
point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [u, v], depth)
# point_3d = [Xc, Yc, Zc] (미터)
print(f"카메라 3D: X={point_3d[0]:.4f}m, Y={point_3d[1]:.4f}m, Z={point_3d[2]:.4f}m")"""

api(f'https://api.notion.com/v1/blocks/{rm_page_id}/children', {
    'children': [
        code_block(code_step1),
    ]
}, method='PATCH')

# Step 2: 캘리브레이션 (SVD)
api(f'https://api.notion.com/v1/blocks/{rm_page_id}/children', {
    'children': [
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
    ]
}, method='PATCH')

# SVD 코드
code_svd = """import numpy as np

def calibrate(camera_points, robot_points):
    \"\"\"
    카메라 3D 좌표 → 로봇 좌표 변환 행렬 계산 (SVD 방식)

    camera_points: np.array (N, 3) - 카메라 좌표 [미터]
    robot_points:  np.array (N, 3) - 로봇 좌표 [밀리미터]
    return: 4x4 변환 행렬 T
    \"\"\"
    # 1. 각 점 집합의 중심(centroid) 계산
    cam_center = camera_points.mean(axis=0)
    rob_center = robot_points.mean(axis=0)

    # 2. 중심 기준으로 이동 (중심을 원점으로)
    cam_centered = camera_points - cam_center
    rob_centered = robot_points - rob_center

    # 3. 공분산 행렬 H 계산
    H = cam_centered.T @ rob_centered  # (3x3)

    # 4. SVD 분해 → 최적 회전 행렬 R
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # 반사(reflection) 보정
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    # 5. 이동 벡터 t 계산
    t = rob_center - R @ cam_center

    # 6. 4x4 동차 변환 행렬 조립
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t

    return T

def camera_to_robot(T, camera_point):
    \"\"\"변환 행렬 T를 사용하여 카메라 좌표 → 로봇 좌표 변환\"\"\"
    p = np.append(camera_point, 1)  # [x, y, z, 1]
    result = T @ p
    return result[:3]  # [Xr, Yr, Zr]"""

api(f'https://api.notion.com/v1/blocks/{rm_page_id}/children', {
    'children': [
        code_block(code_svd),
    ]
}, method='PATCH')

# 캘리브레이션 데이터 수집 예시
code_collect = """# 캘리브레이션 포인트 수집 예시
# 카메라 좌표 (미터) - RealSense에서 측정
camera_points = np.array([
    [0.12, -0.05, 0.85],   # 점 1
    [-0.10, 0.08, 0.82],   # 점 2
    [0.05, 0.15, 0.88],    # 점 3
    [-0.15, -0.10, 0.80],  # 점 4
    [0.20, 0.00, 0.90],    # 점 5
    # ... 최소 5개, 권장 8~10개
])

# 로봇 좌표 (밀리미터) - 로봇 컨트롤러에서 읽기
robot_points = np.array([
    [350.2, -120.5, 150.3],   # 같은 점 1
    [180.1, 95.8, 165.2],     # 같은 점 2
    [280.5, 200.3, 130.1],    # 같은 점 3
    [150.0, -80.2, 180.5],    # 같은 점 4
    [420.3, 10.5, 110.8],     # 같은 점 5
])

# 변환 행렬 계산
T = calibrate(camera_points, robot_points)
print("변환 행렬 T:")
print(T)

# 테스트: 새로운 카메라 좌표를 로봇 좌표로 변환
test_cam = np.array([0.08, 0.03, 0.86])
test_rob = camera_to_robot(T, test_cam)
print(f"카메라: {test_cam} → 로봇: {test_rob}")"""

api(f'https://api.notion.com/v1/blocks/{rm_page_id}/children', {
    'children': [
        code_block(code_collect),
    ]
}, method='PATCH')

# Eye-to-Hand 설명
api(f'https://api.notion.com/v1/blocks/{rm_page_id}/children', {
    'children': [
        heading(2, '2-3. Eye-to-Hand vs Eye-in-Hand'),
        table(3, [
            ['구분', 'Eye-to-Hand (현재 설정)', 'Eye-in-Hand'],
            ['카메라 위치', '로봇 머리(고정 위치)에 장착', '로봇 팔 끝에 장착'],
            ['장점', '캘리브레이션 1회만 하면 됨', '팔이 가까이 가서 정밀도 높음'],
            ['단점', '거리에 따라 정밀도 저하', '팔 움직일 때마다 좌표 재계산'],
            ['적합 상황', '넓은 영역 감시, 고정 작업', '정밀 조립, 좁은 영역'],
        ]),
        callout('현재 프로젝트는 카메라가 로봇 머리에 고정 → Eye-to-Hand 방식. 캘리브레이션을 한 번만 하면 계속 사용 가능.', '✅'),
        divider(),
    ]
}, method='PATCH')

# 캘리브레이션 팁
api(f'https://api.notion.com/v1/blocks/{rm_page_id}/children', {
    'children': [
        heading(2, '2-4. 캘리브레이션 실전 팁'),
        numbered('포인트 분포: 작업 영역 전체에 골고루 분포 (한쪽에 몰리면 정밀도 저하)'),
        numbered('높이 다양성: 같은 높이만 측정하지 말고, 다른 높이의 점도 포함'),
        numbered('단위 주의: 카메라=미터(m), 로봇=밀리미터(mm) → 변환 시 ×1000 필요할 수 있음'),
        numbered('검증: 캘리브레이션 후 테스트 포인트로 오차 확인 (목표: 5mm 이내)'),
        numbered('재캘리브레이션: 카메라 위치가 바뀌면 반드시 다시 수행'),
        divider(),
    ]
}, method='PATCH')

# ──────────────────────────────────────────────────────────────
# Section 3: RealMan SDK
# ──────────────────────────────────────────────────────────────
print('섹션 3: RealMan SDK...')
api(f'https://api.notion.com/v1/blocks/{rm_page_id}/children', {
    'children': [
        heading(1, '3. RealMan 로봇 SDK 정리'),
        heading(2, '3-1. 설치 및 연결'),
    ]
}, method='PATCH')

# SDK 기본 정보 테이블
api(f'https://api.notion.com/v1/blocks/{rm_page_id}/children', {
    'children': [
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
    ]
}, method='PATCH')

# 연결 코드
code_connect = """from Robotic_Arm.rm_robot_interface import *

# 로봇 연결
robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
handle = robot.rm_create_robot_arm("192.168.1.18", 8080)

# 연결 확인 - 소프트웨어 정보 조회
info = robot.rm_get_arm_software_info()
print(f"로봇 정보: {info}")

# 현재 로봇 상태 (관절각 + TCP 위치)
state = robot.rm_get_current_arm_state()
print(f"현재 상태: {state}")
# state에서 얻을 수 있는 정보:
# - 관절각: [j1, j2, j3, j4, j5, j6] (도)
# - TCP 위치: [x, y, z, rx, ry, rz] (mm, 도)

# 연결 해제
robot.rm_delete_robot_arm()"""

api(f'https://api.notion.com/v1/blocks/{rm_page_id}/children', {
    'children': [
        code_block(code_connect),
    ]
}, method='PATCH')

# 이동 명령어
api(f'https://api.notion.com/v1/blocks/{rm_page_id}/children', {
    'children': [
        heading(2, '3-2. 이동 명령어'),
    ]
}, method='PATCH')

api(f'https://api.notion.com/v1/blocks/{rm_page_id}/children', {
    'children': [
        table(4, [
            ['명령어', '타입', '용도', '특징'],
            ['rm_movej', '관절 공간', '빠른 이동, 대기 위치 이동', '최단 경로(관절 기준), 경로 예측 어려움'],
            ['rm_movel', '직선 이동', '정밀 작업, 접근/후퇴', 'TCP가 직선으로 이동, 속도 느림'],
            ['rm_movec', '원호 이동', '곡선 경로 필요 시', '3점(시작/중간/끝)으로 원호 정의'],
            ['rm_movej_p', '관절→포즈', '목표 포즈로 빠른 이동', '관절 공간이지만 목표는 직교 좌표'],
        ]),
    ]
}, method='PATCH')

# 이동 코드
code_move = """# 좌표 형식: [x, y, z, rx, ry, rz]
# x, y, z: 위치 (밀리미터)
# rx, ry, rz: 자세/회전 (라디안 또는 도, 모델에 따라 다름)

# 1. 직선 이동 (정밀 작업용) - 가장 많이 사용
robot.rm_movel(
    [300.0, 0.0, 200.0, 3.14, 0.0, 0.0],  # 목표 좌표
    20    # 속도 (mm/s 또는 %)
)

# 2. 관절 이동 (빠른 이동용)
robot.rm_movej(
    [0.0, -30.0, 60.0, 0.0, 90.0, 0.0],  # 관절각 (j1~j6, 도)
    30    # 속도 (%)
)

# 3. 관절 이동으로 포즈 도달
robot.rm_movej_p(
    [300.0, 0.0, 200.0, 3.14, 0.0, 0.0],  # 목표 직교 좌표
    30    # 속도
)"""

api(f'https://api.notion.com/v1/blocks/{rm_page_id}/children', {
    'children': [
        code_block(code_move),
    ]
}, method='PATCH')

# 그리퍼 제어
api(f'https://api.notion.com/v1/blocks/{rm_page_id}/children', {
    'children': [
        heading(2, '3-3. 그리퍼(Gripper) 제어'),
        paragraph(text('그릇을 잡고 놓기 위한 그리퍼 명령어 (모델에 따라 API 다를 수 있음)')),
    ]
}, method='PATCH')

code_gripper = """# 그리퍼 제어 (RealMan 기본 그리퍼 기준)
# 실제 API는 로봇 모델/그리퍼 타입에 따라 다를 수 있음

# 그리퍼 열기 (그릇 놓기)
robot.rm_set_gripper_release(500)  # 속도

# 그리퍼 닫기 (그릇 잡기)
robot.rm_set_gripper_pick(500, 200)  # 속도, 힘

# IO 기반 그리퍼 제어 (일부 모델)
robot.rm_set_tool_DO(1, True)   # 디지털 출력 1번 ON
robot.rm_set_tool_DO(1, False)  # 디지털 출력 1번 OFF"""

api(f'https://api.notion.com/v1/blocks/{rm_page_id}/children', {
    'children': [
        code_block(code_gripper),
        divider(),
    ]
}, method='PATCH')

# ──────────────────────────────────────────────────────────────
# Section 4: 통합 파이프라인
# ──────────────────────────────────────────────────────────────
print('섹션 4: 통합 파이프라인...')
api(f'https://api.notion.com/v1/blocks/{rm_page_id}/children', {
    'children': [
        heading(1, '4. 전체 통합 파이프라인'),
        callout('Seg 모델 → 카메라 3D → 로봇 좌표 → 이동 명령', '🔗'),
        heading(2, '4-1. 파이프라인 흐름도'),
        numbered('RealSense 컬러+깊이 프레임 획득'),
        numbered('YOLOv8 Seg 모델로 그릇 마스크 감지 (conf 0.7+)'),
        numbered('마스크 중심점 (cx, cy) 계산 → 픽셀 좌표'),
        numbered('RealSense 깊이로 (cx, cy) → 카메라 3D 좌표 변환'),
        numbered('캘리브레이션 행렬로 카메라 3D → 로봇 좌표 변환'),
        numbered('로봇에 movel 명령 전송 → 그릇 위치로 이동'),
        numbered('그리퍼로 그릇 잡기 → 목표 위치로 이동 → 놓기'),
    ]
}, method='PATCH')

# 통합 코드 (파트 1)
code_pipeline_1 = """import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
from Robotic_Arm.rm_robot_interface import *

# ─── 초기화 ─────────────────────────────────
# 1. RealSense 카메라
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)
align = rs.align(rs.stream.color)

# 2. YOLOv8 Seg 모델
model = YOLO('runs/segment/bowl_seg_v3/weights/best.pt')

# 3. 로봇 연결
robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
robot.rm_create_robot_arm("192.168.1.18", 8080)

# 4. 캘리브레이션 행렬 (사전에 계산해둔 것)
T = np.load('calibration_matrix.npy')  # 4x4 변환 행렬"""

code_pipeline_2 = """# ─── 메인 루프 ────────────────────────────────
def detect_and_pick():
    # 1. 프레임 획득
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    color_image = np.asanyarray(color_frame.get_data())
    intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

    # 2. Seg 모델 추론
    results = model(color_image, conf=0.7)

    for r in results:
        if r.masks is None:
            continue
        for mask in r.masks.xy:
            # 3. 마스크 중심점 계산
            cx = int(mask[:, 0].mean())
            cy = int(mask[:, 1].mean())

            # 4. 픽셀 → 카메라 3D
            depth = depth_frame.get_distance(cx, cy)
            if depth < 0.1 or depth > 1.5:  # 유효 깊이 필터
                continue
            cam_3d = rs.rs2_deproject_pixel_to_point(
                intrinsics, [cx, cy], depth)

            # 5. 카메라 3D → 로봇 좌표
            cam_point = np.array(cam_3d)
            rob_point = T[:3,:3] @ cam_point + T[:3,3]

            # 6. 로봇 이동 (접근 → 잡기 → 이동 → 놓기)
            x, y, z = rob_point

            # 접근 (위에서 내려가기)
            robot.rm_movel([x, y, z+50, 3.14, 0, 0], 30)
            # 잡기 위치로 하강
            robot.rm_movel([x, y, z, 3.14, 0, 0], 10)
            # 그리퍼 잡기
            robot.rm_set_gripper_pick(500, 200)
            # 들어올리기
            robot.rm_movel([x, y, z+100, 3.14, 0, 0], 20)
            # 목표 위치로 이동
            robot.rm_movel([400, 0, 200, 3.14, 0, 0], 30)
            # 그리퍼 놓기
            robot.rm_set_gripper_release(500)

            print(f"그릇 이동 완료: ({x:.1f}, {y:.1f}, {z:.1f})")
            return True
    return False"""

api(f'https://api.notion.com/v1/blocks/{rm_page_id}/children', {
    'children': [
        heading(2, '4-2. 통합 코드 예시'),
        paragraph(text('실제 사용 시 로봇 모델/그리퍼에 맞게 수정 필요', italic=True, color='gray')),
        code_block(code_pipeline_1),
        code_block(code_pipeline_2),
        divider(),
    ]
}, method='PATCH')

# ──────────────────────────────────────────────────────────────
# Section 5: Phase 1 구현 가이드
# ──────────────────────────────────────────────────────────────
print('섹션 5: Phase 1 구현 가이드...')
api(f'https://api.notion.com/v1/blocks/{rm_page_id}/children', {
    'children': [
        heading(1, '5. Phase 1 구현 가이드 (고정 위치 Pick & Place)'),
        callout('카메라/Seg 모델 없이, 하드코딩된 좌표로 로봇 제어 먼저 완성', '🎯'),
        heading(2, '5-1. 구현 순서'),
        numbered('로봇 연결 테스트: ping → SDK 연결 → 상태 조회'),
        numbered('홈 위치 설정: 안전한 대기 포즈 정의'),
        numbered('좌표 티칭: 거치대 위 그릇 위치를 직접 기록 (교시 모드)'),
        numbered('Pick 동작: 접근 → 하강 → 잡기 → 상승'),
        numbered('Place 동작: 이동 → 하강 → 놓기 → 상승'),
        numbered('루프: 여러 그릇 순차 이동'),
    ]
}, method='PATCH')

# Phase 1 코드
code_phase1 = """from Robotic_Arm.rm_robot_interface import *
import time

robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
robot.rm_create_robot_arm("192.168.1.18", 8080)

# ─── 좌표 정의 (티칭으로 미리 기록) ──────────
HOME = [0.0, -30.0, 60.0, 0.0, 90.0, 0.0]  # 관절각 (홈 위치)

# 거치대 위 그릇 위치들 [x, y, z, rx, ry, rz] (mm, rad)
BOWL_POSITIONS = [
    [300.0, -100.0, 150.0, 3.14, 0.0, 0.0],  # 1번 그릇
    [300.0,    0.0, 150.0, 3.14, 0.0, 0.0],  # 2번 그릇
    [300.0,  100.0, 150.0, 3.14, 0.0, 0.0],  # 3번 그릇
]

# 목표 위치 (그릇을 놓을 곳)
PLACE_POSITION = [400.0, 200.0, 150.0, 3.14, 0.0, 0.0]

# 접근 높이 (그릇 위 50mm에서 시작)
APPROACH_HEIGHT = 50  # mm

# ─── Pick & Place 함수 ──────────────────────
def pick(pos):
    x, y, z, rx, ry, rz = pos
    # 1. 위에서 접근
    robot.rm_movel([x, y, z + APPROACH_HEIGHT, rx, ry, rz], 30)
    time.sleep(0.5)
    # 2. 하강
    robot.rm_movel(pos, 10)
    time.sleep(0.5)
    # 3. 잡기
    robot.rm_set_gripper_pick(500, 200)
    time.sleep(1.0)
    # 4. 상승
    robot.rm_movel([x, y, z + APPROACH_HEIGHT, rx, ry, rz], 20)
    time.sleep(0.5)

def place(pos):
    x, y, z, rx, ry, rz = pos
    robot.rm_movel([x, y, z + APPROACH_HEIGHT, rx, ry, rz], 30)
    time.sleep(0.5)
    robot.rm_movel(pos, 10)
    time.sleep(0.5)
    robot.rm_set_gripper_release(500)
    time.sleep(1.0)
    robot.rm_movel([x, y, z + APPROACH_HEIGHT, rx, ry, rz], 20)
    time.sleep(0.5)

# ─── 메인 실행 ──────────────────────────────
robot.rm_movej(HOME, 30)  # 홈 위치로
time.sleep(1)

for i, bowl_pos in enumerate(BOWL_POSITIONS):
    print(f"그릇 {i+1} 이동 중...")
    pick(bowl_pos)
    place(PLACE_POSITION)
    robot.rm_movej(HOME, 30)  # 홈으로 복귀
    time.sleep(0.5)

print("모든 그릇 이동 완료!")
robot.rm_delete_robot_arm()"""

api(f'https://api.notion.com/v1/blocks/{rm_page_id}/children', {
    'children': [
        heading(2, '5-2. Phase 1 예시 코드'),
        code_block(code_phase1),
        divider(),
    ]
}, method='PATCH')

# ──────────────────────────────────────────────────────────────
# Section 6: 참고 링크
# ──────────────────────────────────────────────────────────────
print('섹션 6: 참고 링크...')
api(f'https://api.notion.com/v1/blocks/{rm_page_id}/children', {
    'children': [
        heading(1, '6. 참고 링크 및 리소스'),
        bullet('RealMan Python SDK 문서: https://develop.realman-robotics.com/en/robot/apipython/getStarted/'),
        bullet('RealMan Demo 예제: https://develop.realman-robotics.com/en/robot/demo/python/simpleProcess/'),
        bullet('RealMan GitHub: https://github.com/RealManRobot/RM_API2'),
        bullet('RealSense SDK: https://github.com/IntelRealSense/librealsense'),
        bullet('YOLOv8 문서: https://docs.ultralytics.com/'),
        divider(),
        heading(1, '7. 주의사항'),
        callout('로봇 동작 테스트 시 반드시 비상정지 버튼을 손에 쥐고 진행!', '⚠️'),
        numbered('처음 테스트는 반드시 저속(10% 이하)으로 진행'),
        numbered('작업 영역 내 사람이 없는지 확인'),
        numbered('관절 한계(joint limit) 초과하지 않도록 좌표 범위 확인'),
        numbered('그리퍼 힘 조절: 그릇이 깨지지 않을 정도로 (도자기 주의)'),
        numbered('캘리브레이션 후 검증 포인트로 오차 반드시 확인'),
    ]
}, method='PATCH')

print('\n완료!')
print('프로젝트 진행 일지')
print('  ├── Samsung Wellstory')
print('  └── RealMan 로봇 제어')
print('       ├── 1. 프로젝트 전략 (1차 vs 최종 목표)')
print('       ├── 2. 카메라-로봇 캘리브레이션')
print('       ├── 3. RealMan SDK 정리')
print('       ├── 4. 전체 통합 파이프라인')
print('       ├── 5. Phase 1 구현 가이드')
print('       ├── 6. 참고 링크')
print('       └── 7. 주의사항')
