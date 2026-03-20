import json, urllib.request, sys
sys.stdout.reconfigure(encoding='utf-8')

API_KEY = 'ntn_b57077336502qA3Deg7jq4qWxqnc8navEmCPcNhjyv7gn7'
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
        print(f'ERROR {e.code}: {e.read().decode("utf-8")[:300]}')
        raise

def block(type_, **kwargs):
    return {'object': 'block', 'type': type_, type_: kwargs}

def heading(level, text):
    t = f'heading_{level}'
    return block(t, rich_text=[{'type': 'text', 'text': {'content': text}}])

def para(text):
    return block('paragraph', rich_text=[{'type': 'text', 'text': {'content': text}}])

def code_block(text, lang='python'):
    return block('code', rich_text=[{'type': 'text', 'text': {'content': text[:1999]}}], language=lang)

def bullet(text):
    return block('bulleted_list_item', rich_text=[{'type': 'text', 'text': {'content': text}}])

def divider():
    return {'object': 'block', 'type': 'divider', 'divider': {}}

def add_blocks(page_id, blocks):
    for i in range(0, len(blocks), 90):
        chunk = blocks[i:i+90]
        api(f'https://api.notion.com/v1/blocks/{page_id}/children', {'children': chunk}, method='PATCH')
        print(f'  블록 {i+1}~{i+len(chunk)} 추가 완료')

# ─── 기존 Code Review 페이지에 step3 서브페이지 생성 ───
CR_PAGE_ID = '32815435-1a92-8171-9f9e-de8228ed1f54'  # Code Review 상위 페이지

print('step3_calibration.py 페이지 생성 중...')
step3_page = api('https://api.notion.com/v1/pages', {
    'parent': {'page_id': CR_PAGE_ID},
    'icon': {'type': 'emoji', 'emoji': '📐'},
    'properties': {
        'title': {'title': [{'text': {'content': 'step3_calibration.py'}}]}
    },
}, method='POST')
step3_id = step3_page['id']
print(f'  step3 페이지 생성: {step3_id}')

# ─── 코드 분할 (2000자 이하) ───
code_part1 = '''\
"""
3단계: Eye-to-Hand 캘리브레이션
- 고정 카메라 ↔ 로봇 베이스 간의 변환 행렬(T) 계산
- 로봇 TCP를 여러 위치로 이동 → 카메라에서 클릭 → 좌표 쌍 수집
- 최소 5개, 권장 8~10개 포인트 수집
키: Space=저장 | c=계산 | x=오차큰것삭제 | z=특정포인트삭제 | s=저장 | q=종료
"""
import sys, os, json, math
import numpy as np
import cv2
import pyrealsense2 as rs
from datetime import datetime

SAVE_DIR = r'C:\...\calibration'   # calibration_matrix.npy 저장 위치

# ── 로봇 연결 ──────────────────────────────────────────────
robot = Arm(RM65, "192.168.1.18")  # TCP/IP 연결

# ── RealSense D435i 설정 ───────────────────────────────────
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
profile = pipeline.start(config)

# 레이저 파워 최대로 설정 (depth 정확도 향상)
depth_sensor = profile.get_device().first_depth_sensor()
max_laser = depth_sensor.get_option_range(rs.option.laser_power).max
depth_sensor.set_option(rs.option.laser_power, max_laser)

# depth 필터 (노이즈 제거)
spatial = rs.spatial_filter()    # 공간 필터
temporal = rs.temporal_filter()  # 시간 필터
hole_filling = rs.hole_filling_filter()  # 홀 채우기
for _ in range(60):              # 60프레임 안정화 (auto-exposure 수렴)
    pipeline.wait_for_frames()
'''

code_part2 = '''\
# ── 수집 데이터 저장소 ──────────────────────────────────────
camera_points = []   # 카메라 3D 좌표 목록 (mm)
robot_points  = []   # 로봇 TCP 좌표 목록 (mm)
pending_click = None # 클릭했지만 아직 Space 안 누른 임시 포인트
last_errors   = None # 마지막 캘리브레이션 오차 (x키에서 사용)

# ── 핵심 함수: SVD로 변환 행렬 계산 ────────────────────────
def compute_calibration():
    cam = np.array(camera_points, dtype=np.float64)  # Nx3 카메라 좌표
    rob = np.array(robot_points,  dtype=np.float64)  # Nx3 로봇 좌표

    # 두 점군의 무게중심(평균)을 원점으로 이동 (중심화)
    cam_center = cam.mean(axis=0)
    rob_center = rob.mean(axis=0)
    cam_c = cam - cam_center
    rob_c = rob - rob_center

    # SVD(특이값 분해)로 최적 회전 행렬 R 계산
    # H = 카메라 점군 ↔ 로봇 점군의 상관관계 행렬
    H = cam_c.T @ rob_c
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # reflection 보정 (det(R) = -1이면 뒤집힌 것 → 보정)
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    # 이동 벡터 t 계산 (회전 후 무게중심 차이)
    t = rob_center - R @ cam_center

    # 4x4 변환 행렬 T 조립 [R|t; 0|1]
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3]  = t

    # 오차 계산: 각 포인트에서 예측값과 실제값의 차이
    errors = [np.linalg.norm(R @ cam[i] + t - rob[i]) for i in range(len(cam))]
    avg_err = np.mean(errors)

    np.save(os.path.join(SAVE_DIR, \'calibration_matrix.npy\'), T)
    return T, errors
'''

code_part3 = '''\
# ── 클릭 처리: 빨간 마커 자동 보정 ────────────────────────
if click_pos:
    cx, cy = click_pos
    crop_r = 80  # 클릭 주변 160x160 픽셀 영역만 분석

    # 영역 잘라내기
    crop = color_image[cy-crop_r:cy+crop_r, cx-crop_r:cx+crop_r]

    # HSV 색공간으로 변환 후 빨간색 영역 검출
    # 빨간색은 H가 0 근처(0~10)와 180 근처(170~180) 두 곳에 분포
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, (0,  30, 50), (10,  255, 255))  # H=0~10
    mask2 = cv2.inRange(hsv, (170, 30, 50), (180, 255, 255)) # H=170~180
    binary = mask1 | mask2

    # 모폴로지 연산으로 작은 노이즈 제거
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)   # 침식→팽창
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)  # 팽창→침식

    # 윤곽선에서 원형(볼트 구멍)에 가장 가까운 것 선택
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        perimeter = cv2.arcLength(cnt, True)
        # 원형도 = 4π × 면적 / 둘레² → 1에 가까울수록 원
        circularity = 4 * np.pi * area / (perimeter**2)
        if circularity > 0.5:   # 원에 가까운 것만
            M = cv2.moments(cnt)
            if M["m00"] > 0:
                cx = int(M["m10"]/M["m00"]) + (cx - crop_r)  # 전체 이미지 좌표로
                cy = int(M["m01"]/M["m00"]) + (cy - crop_r)
'''

code_part4 = '''\
# ── depth 측정 (5x5 중앙값으로 노이즈 방지) ───────────────
    radius = 2
    depths = []
    for dy in range(-radius, radius+1):
        for dx in range(-radius, radius+1):
            dd = depth_filtered.get_distance(cx+dx, cy+dy)
            if dd > 0.2:
                depths.append(dd)
    d = float(np.median(depths))  # 중앙값 사용 (이상치 제거)

    # 픽셀+depth → 3D 카메라 좌표 (역투영)
    p = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], d)
    pending_click = (cx, cy, [p[0]*1000, p[1]*1000, p[2]*1000])

# ── Space키: 포인트 저장 ──────────────────────────────────
elif key == ord(\' \') and pending_click:
    _, _, cam_3d = pending_click
    ret = robot.Get_Current_Arm_State(retry=1)
    tcp = ret[2]  # [x, y, z, rx, ry, rz] (미터)
    tcp_xyz = [tcp[0]*1000, tcp[1]*1000, tcp[2]*1000]  # mm로 변환
    camera_points.append(cam_3d)
    robot_points.append(tcp_xyz)
    # 카메라로 클릭한 위치와 로봇 TCP가 같은 물리적 점이라고 가정 → 쌍 저장

# ── c키: 캘리브레이션 계산 ────────────────────────────────
elif key == ord(\'c\'):
    T, last_errors = compute_calibration()
    # → calibration_matrix.npy 저장됨

# ── x키: 오차 10mm 이상 포인트 자동 삭제 후 재계산 ────────
elif key == ord(\'x\'):
    bad_indices = [i for i, e in enumerate(last_errors) if e >= 10]
    for idx in sorted(bad_indices, reverse=True):  # 역순 삭제 (인덱스 안 꼬임)
        camera_points.pop(idx)
        robot_points.pop(idx)
    T, last_errors = compute_calibration()  # 재계산
'''

# ─── 설명 블록 구성 ───
blocks_1 = [
    heading(1, '📐 Code Review — step3_calibration.py'),
    para('Eye-to-Hand 캘리브레이션 스크립트입니다. 카메라와 로봇이 서로 다른 좌표계를 사용하기 때문에, '
         '"같은 물리적 점"의 카메라 좌표와 로봇 좌표를 여러 개 수집해서 변환 행렬을 계산합니다. '
         '이 행렬이 calibration_matrix.npy로 저장되고, step7에서 접시 좌표 변환에 사용됩니다.'),
    divider(),
    heading(2, '📄 전체 코드'),
    para('※ 가독성을 위해 설명 주석이 추가된 버전입니다. 실제 코드는 calibration/step3_calibration.py 참고'),
    code_block(code_part1),
    code_block(code_part2),
    code_block(code_part3),
    code_block(code_part4),
]

blocks_2 = [
    divider(),
    heading(2, '📚 코드 상세 설명'),

    heading(3, '1. 전체 동작 흐름'),
    para('캘리브레이션은 "같은 점을 카메라와 로봇이 각자 어떻게 보는지" 기록하는 작업입니다:'),
    bullet('로봇을 수동으로 특정 위치로 이동 (webapp 또는 직접 관절 제어)'),
    bullet('카메라 화면에서 플랜지 볼트 중심을 마우스로 클릭 → 카메라 3D 좌표 계산'),
    bullet('Space키 → 카메라 좌표 + 로봇 TCP 좌표를 한 쌍으로 저장'),
    bullet('위 과정을 8~10번 반복 (다양한 위치에서, 넓게 분포되도록)'),
    bullet('c키 → SVD로 변환 행렬 계산 → calibration_matrix.npy 저장'),
    bullet('x키 → 오차 10mm 이상 포인트 자동 삭제 후 재계산 (정확도 향상)'),
    divider(),

    heading(3, '2. Eye-to-Hand 캘리브레이션이란?'),
    para('카메라가 로봇에 붙어있지 않고 고정된 위치에 있는 방식입니다 (Eye-in-Hand는 카메라가 팔에 붙음):'),
    bullet('카메라 좌표계: 카메라 렌즈 중심이 원점, Z축이 카메라가 바라보는 방향'),
    bullet('로봇 좌표계: 로봇 베이스(바닥) 중심이 원점, Z축이 위쪽'),
    bullet('두 좌표계는 완전히 다르므로 변환 행렬 T가 필요'),
    bullet('T = [R|t] : R은 회전(방향 맞추기), t는 이동(거리 맞추기)'),
    bullet('결과: 카메라가 "접시가 (200, 150, 600)mm에 있다"고 하면 → T를 곱해 로봇 좌표로 변환'),
    divider(),

    heading(3, '3. SVD 기반 변환 행렬 계산 (compute_calibration 함수)'),
    para('여러 좌표 쌍에서 가장 오차가 적은 변환 행렬을 수학적으로 찾습니다:'),
    bullet('Step 1: 수집된 카메라 좌표들의 무게중심(평균)과 로봇 좌표들의 무게중심 계산'),
    bullet('Step 2: 두 점군을 각자 원점으로 이동 (중심화) → 순수 회전만 남김'),
    bullet('Step 3: H = 카메라점군.T @ 로봇점군 → 상관관계 행렬'),
    bullet('Step 4: SVD(H) = U, S, Vt → R = Vt.T @ U.T (최적 회전 행렬)'),
    bullet('Step 5: t = 로봇무게중심 - R @ 카메라무게중심 (이동 벡터)'),
    bullet('Step 6: 오차 검증 → 각 포인트에서 예측값과 실제 로봇 좌표 차이 계산'),
    bullet('등급: 평균 5mm 이내=우수, 10mm=양호, 20mm=보통, 그 이상=불량'),
]

blocks_3 = [
    divider(),

    heading(3, '4. 빨간 마커 자동 보정 (HSV 검출)'),
    para('대충 클릭해도 플랜지 볼트의 정확한 중심을 자동으로 찾아줍니다:'),
    bullet('클릭 주변 160x160 영역만 잘라서 분석 (전체 이미지 분석하면 느림)'),
    bullet('BGR → HSV 색공간 변환 (HSV가 조명 변화에 강함)'),
    bullet('빨간색은 H값이 0~10 AND 170~180 두 범위 (색상환에서 0과 360이 같은 빨강)'),
    bullet('S(채도) 임계값 30: 조명이 바뀌어도 인식되도록 낮게 설정 (원래 50 → 30으로 조정)'),
    bullet('모폴로지 Open: 작은 노이즈 제거 / Close: 구멍 채우기'),
    bullet('원형도(circularity) > 0.5인 윤곽선만 선택 → 볼트 구멍 모양에 가까운 것'),
    bullet('검출 실패 시: 클릭한 위치 그대로 사용'),
    divider(),

    heading(3, '5. depth 측정 방법 (5x5 중앙값)'),
    para('단일 픽셀 depth는 노이즈가 많아서 주변 5x5=25개 픽셀의 중앙값을 사용합니다:'),
    bullet('중앙값(median): 이상치(튀는 값)에 강함 → 평균보다 안정적'),
    bullet('유효 depth 기준: 0.2m 이상 (너무 가까우면 RealSense가 측정 못함)'),
    bullet('유효 depth가 5개 미만이면 "다시 클릭하세요" 경고'),
    bullet('rs2_deproject_pixel_to_point: 픽셀(x,y) + depth → 3D 카메라 좌표(X,Y,Z)'),
    divider(),

    heading(3, '6. x키: 오차 큰 포인트 자동 삭제'),
    para('c키로 계산 후 x키를 누르면 정확도가 낮은 포인트를 자동으로 찾아 제거합니다:'),
    bullet('오차 10mm 이상인 포인트 번호 목록 추출'),
    bullet('역순으로 삭제 (앞에서부터 지우면 뒤 인덱스가 꼬이므로 뒤에서부터)'),
    bullet('삭제 후 남은 포인트로 자동 재계산'),
    bullet('포인트가 5개 미만이 되면 추가 수집 필요 → 다시 로봇 이동 + 클릭'),
    divider(),

    heading(3, '7. 정확도를 높이는 팁'),
    para('캘리브레이션 품질은 포인트 수집 방법에 크게 좌우됩니다:'),
    bullet('포인트를 넓게 분포 (작업 영역 전체에 골고루, 한쪽에 몰리지 않게)'),
    bullet('권장 7~10개 포인트 (5개는 최소값, 정확도 낮음)'),
    bullet('각 포인트마다 로봇 자세(각도)도 다양하게 변경'),
    bullet('카메라 위치 변경 시마다 재수행 필요 → 매일 세팅 시 진행'),
    bullet('목표: 평균 오차 5mm 이내 (현재 달성: avg 5.3mm / max 7.8mm)'),
    bullet('x키로 오차 큰 포인트 제거 후 재계산하면 정확도 향상'),
]

# ─── 업로드 ───
print('\n블록 추가 중 (1/3)...')
add_blocks(step3_id, blocks_1)
print('블록 추가 중 (2/3)...')
add_blocks(step3_id, blocks_2)
print('블록 추가 중 (3/3)...')
add_blocks(step3_id, blocks_3)

print('\n✅ 완료!')
print(f'💻 Code Review')
print(f'  ├── 🤖 step7_twoplate_alwaysdetect.py')
print(f'  └── 📐 step3_calibration.py  ← 새로 추가')
