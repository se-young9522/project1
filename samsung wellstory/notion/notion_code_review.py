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

def para(text, bold=False, color='default'):
    ann = {}
    if bold:
        ann['bold'] = True
    return block('paragraph', rich_text=[{
        'type': 'text',
        'text': {'content': text},
        'annotations': ann if ann else {}
    }])

def code_block(text, lang='python'):
    # 2000자 제한
    text = text[:1999]
    return block('code', rich_text=[{'type': 'text', 'text': {'content': text}}], language=lang)

def bullet(text):
    return block('bulleted_list_item', rich_text=[{'type': 'text', 'text': {'content': text}}])

def divider():
    return {'object': 'block', 'type': 'divider', 'divider': {}}

def add_blocks(page_id, blocks):
    # Notion API는 한번에 100개 블록까지
    for i in range(0, len(blocks), 90):
        chunk = blocks[i:i+90]
        api(f'https://api.notion.com/v1/blocks/{page_id}/children', {'children': chunk}, method='PATCH')
        print(f'  블록 {i+1}~{i+len(chunk)} 추가 완료')

# ─── Samsung Wellstory 프로젝트 페이지 찾기 ───
print('Samsung Wellstory 페이지 검색 중...')
search = api('https://api.notion.com/v1/search', {
    'query': 'Samsung Wellstory',
    'filter': {'property': 'object', 'value': 'page'},
}, method='POST')

sw_page_id = None
for page in search.get('results', []):
    title = ''
    for t in page.get('properties', {}).get('title', {}).get('title', []):
        title += t.get('plain_text', '')
    if 'Samsung Wellstory' in title or 'samsung wellstory' in title.lower():
        sw_page_id = page['id']
        print(f'  발견: {title} ({sw_page_id})')
        break

if not sw_page_id:
    # 프로젝트 진행 일지에서 찾기
    search2 = api('https://api.notion.com/v1/search', {
        'query': '프로젝트 진행 일지',
        'filter': {'property': 'object', 'value': 'page'},
    }, method='POST')
    for page in search2.get('results', []):
        title = ''
        for t in page.get('properties', {}).get('title', {}).get('title', []):
            title += t.get('plain_text', '')
        print(f'  후보: {title}')
        if '삼성' in title or 'wellstory' in title.lower() or 'samsung' in title.lower():
            sw_page_id = page['id']
            break

if not sw_page_id:
    print('Samsung Wellstory 페이지를 찾을 수 없습니다. page_id를 직접 입력하세요.')
    sw_page_id = '32015435-1a92-813d-a3f0-e433e57bc50a'  # 기존 프로젝트 페이지 ID
    print(f'  기본값 사용: {sw_page_id}')

# ─── 기존 Code Review 페이지들 아카이브 ───
OLD_IDS = [
    '32815435-1a92-8152-a4d2-f6bbf711d5b8',  # 1차
    '32815435-1a92-817e-bedd-d448427e01de',  # 2차
]
print(f'\n기존 Code Review 페이지 아카이브 중...')
for oid in OLD_IDS:
    try:
        api(f'https://api.notion.com/v1/pages/{oid}', {'archived': True}, method='PATCH')
        print(f'  아카이브 완료: {oid}')
    except Exception as e:
        print(f'  스킵: {oid}')

# ─── Code Review 상위 페이지 생성 ───
print('\nCode Review 상위 페이지 생성 중...')
cr_page = api('https://api.notion.com/v1/pages', {
    'parent': {'page_id': sw_page_id},
    'icon': {'type': 'emoji', 'emoji': '💻'},
    'properties': {
        'title': {'title': [{'text': {'content': 'Code Review'}}]}
    },
}, method='POST')
cr_id = cr_page['id']
print(f'  Code Review 페이지 생성: {cr_id}')

# ─── step7 서브페이지 생성 ───
print('\nstep7_twoplate_alwaysdetect.py 페이지 생성 중...')
step7_page = api('https://api.notion.com/v1/pages', {
    'parent': {'page_id': cr_id},
    'icon': {'type': 'emoji', 'emoji': '🤖'},
    'properties': {
        'title': {'title': [{'text': {'content': 'step7_twoplate_alwaysdetect.py'}}]}
    },
}, method='POST')
step7_id = step7_page['id']
print(f'  step7 페이지 생성: {step7_id}')

# ─── 코드 분할 (2000자 이하) ───
code_part1 = '''\
"""
다중 접시 순차 Pick & Place 테스트
- YOLO로 모든 접시 인식 → Start Point 기준 가까운 순 정렬
- Start → Bowl1 → Start → Bowl2 → Start ... 순차 이동
키 조작: s=Start저장 / d=접시인식 / g=연속루프시작 / x=루프중지 / q=종료
"""
import sys, os, math
import numpy as np
import cv2
import pyrealsense2 as rs

# ── 설정값 (여기서 수치를 바꿔서 조정) ──────────────────
CALIB_DIR = r\'C:\\...\\calibration\'          # 캘리브레이션 파일 폴더
MODEL_PATH = r\'C:\\...\\bowl_seg_v53\\...\\best.pt\'  # YOLO 모델 경로
SAFE_HEIGHT = 200        # 접시 위 몇 mm에서 멈출지 (안전 높이)
CAMERA_TILT_DEG = 55.0   # 카메라 기울기 각도 (수직 기준)
OFFSET_X_MM = -20        # X축 오프셋 보정값 (카메라 위치 편향 보정)
OFFSET_Y_MM = 25         # Y축 오프셋 보정값
V_MOVE = 100             # 로봇 이동 속도

# ── 캘리브레이션 행렬 로드 ─────────────────────────────
# calibration_matrix.npy = 카메라좌표 → 로봇좌표 변환 행렬 (step3에서 생성)
T = np.load(os.path.join(CALIB_DIR, \'calibration_matrix.npy\'))
R = T[:3, :3]   # 회전 행렬 (3x3)
t = T[:3, 3]    # 이동 벡터 (3x1)
'''

code_part2 = '''\
# ── 핵심 함수 1: 카메라 좌표 → 로봇 좌표 변환 ──────────
def cam_to_robot(cam_xyz_mm):
    cam = np.array(cam_xyz_mm)
    rob = R @ cam + t        # 행렬 곱: 회전 + 이동
    rob[0] += OFFSET_X_MM   # X 오프셋 보정 (카메라 위치에 따라 조정 필요)
    rob[1] += OFFSET_Y_MM   # Y 오프셋 보정
    return rob.tolist()

# ── 핵심 함수 2: Start Point까지 거리 계산 ──────────────
def dist_from_start(rob_xyz):
    # 접시가 여러 개일 때 가까운 순으로 정렬하기 위한 함수
    if START_TCP_XY is None:
        return 0
    dx = rob_xyz[0] - START_TCP_XY[0]
    dy = rob_xyz[1] - START_TCP_XY[1]
    return math.sqrt(dx*dx + dy*dy)

# ── 핵심 함수 3: 접시 중심 3D 좌표 계산 ────────────────
def detect_bowl_center(mask_data, depth_filtered, intrinsics):
    # Step1: YOLO 마스크 → 윤곽선 추출
    mask_resized = cv2.resize(mask_data, (1280, 720))
    mask_uint8 = (mask_resized > 0.5).astype(np.uint8) * 255
    contours_bowl, _ = cv2.findContours(mask_uint8,
                           cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours_bowl: return None, None, None
    largest = max(contours_bowl, key=cv2.contourArea)
    if len(largest) < 5: return None, None, None
'''

code_part3 = '''\
    # Step2: 타원 피팅 → 접시의 픽셀 중심 계산
    # (접시가 카메라에 비스듬히 보여서 원이 아닌 타원으로 보임)
    ell = cv2.fitEllipse(largest)
    ecx, ecy = int(ell[0][0]), int(ell[0][1])  # 타원 중심 픽셀

    # Step3: 접시 테두리(rim)의 depth 측정
    # (접시 가장자리 윤곽점에서 depth 값 샘플링 → 25퍼센타일 사용)
    rim_depths = []
    contour_pts = largest.reshape(-1, 2)
    step = max(1, len(contour_pts) // 40)
    for pt in contour_pts[::step]:
        dd = depth_filtered.get_distance(pt[0], pt[1])
        if dd > 0.1:
            rim_depths.append(dd)
    if len(rim_depths) < 5: return None, None, None
    rim_depth = np.percentile(rim_depths, 25)  # 가장 가까운 쪽 depth

    # Step4: 픽셀+depth → 3D 카메라 좌표 변환 (역투영)
    p3 = rs.rs2_deproject_pixel_to_point(intrinsics, [ecx, ecy], rim_depth)
    center_3d = np.array([p3[0]*1000, p3[1]*1000, p3[2]*1000])

    # Step5: 카메라 55° 기울기로 인한 중심 편향 보정
    bowl_depth_mm = 30   # 접시 깊이 가정값
    tilt_rad = np.deg2rad(CAMERA_TILT_DEG)
    shift_mm = bowl_depth_mm * np.sin(tilt_rad) * 0.5
    center_3d[1] -= shift_mm * np.cos(tilt_rad)
    center_3d[2] -= shift_mm * np.sin(tilt_rad)
    return center_3d, ecx, ecy
'''

code_part4 = '''\
# ── 모델/로봇/카메라 초기화 ────────────────────────────
model = YOLO(MODEL_PATH)          # YOLO 세그멘테이션 모델 로드
robot = Arm(RM65, "192.168.1.18") # 로봇팔 TCP/IP 연결
# 로봇 엔드툴이 아래를 향할 때의 자세값 (rx,ry,rz = 오일러각)
DOWN_RX, DOWN_RY, DOWN_RZ = 3.103, 0.051, 0.661
MIN_Z = 0.080  # 최소 이동 높이 80mm (이하로 내려가지 않음)

# RealSense D435i 카메라 설정
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
pipeline.start(config)
align = rs.align(rs.stream.color)  # depth를 color 해상도에 맞춤
spatial = rs.spatial_filter()      # 공간 노이즈 제거 필터
temporal = rs.temporal_filter()    # 시간 노이즈 제거 필터
for _ in range(30):                # 30프레임 워밍업 (auto-exposure 수렴)
    pipeline.wait_for_frames()

# 전역 상태 변수
START_JOINTS = None   # 로봇 시작 관절각 (6개 값)
START_TCP_XY = None   # 시작 위치 XY (접시까지 거리 계산용)
detected_bowls = []   # 인식된 접시 리스트
'''

code_part5 = '''\
# ── 메인 루프 (카메라 실시간 표시 + 키 입력 처리) ─────
try:
    while True:
        # 프레임 획득 + depth 필터 적용
        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)
        depth_filtered = temporal.process(spatial.process(
                            aligned.get_depth_frame())).as_depth_frame()
        color_image = np.asanyarray(aligned.get_color_frame().get_data())
        display = color_image.copy()

        # 마우스 위치의 카메라/로봇 좌표 실시간 표시
        depth_m = depth_filtered.get_distance(mouse_x, mouse_y)
        point_3d = rs.rs2_deproject_pixel_to_point(intrinsics,
                        [mouse_x, mouse_y], depth_m)
        cam_mm = [p*1000 for p in point_3d]
        rob_mm = cam_to_robot(cam_mm)
        cv2.putText(display, f"Camera: {cam_mm}", (10,30), ...)
        cv2.putText(display, f"Robot: {rob_mm}", (10,55), ...)

        # 인식된 접시들 화면에 번호 표시 (#1=빨강, #2+=주황)
        for idx, (cam_xyz, rob_xyz, bcx, bcy) in enumerate(detected_bowls):
            color = (0,0,255) if idx == 0 else (0,165,255)
            cv2.circle(display, (bcx, bcy), 15, color, 3)
            cv2.putText(display, f"#{idx+1}", ...)

        key = cv2.waitKey(1) & 0xFF
        # s: Start Point 저장 / d: 접시 인식 / g: 루프 시작 / q: 종료
'''

code_part6 = '''\
        # ── g키: 연속 루프 (핵심 동작) ────────────────────
        elif key == ord(\'g\'):
            running = True
            last_visited_rob_xyz = None  # 직전 방문 접시 위치 기억용
            cycle = 0

            while running:
                cycle += 1
                if cv2.waitKey(100) & 0xFF == ord(\'x\'):
                    break  # x키로 루프 중지

                # 매 사이클: Start Point에서 접시 재인식
                # (접시가 제거됐으면 자동으로 감지)
                bowls_found = [detect all bowls in current frame]

                if not bowls_found:
                    # 접시 없음 → 대기 루프 (접시 놓으면 자동 재개)
                    while running:
                        cv2.putText(img2, "WAITING FOR BOWLS...", ...)
                        bowls_found = [detect again]
                        if bowls_found:
                            last_visited_rob_xyz = None  # 초기화
                            break

                # 직전 방문 접시 제외하고 선택 (교대 방문)
                # ex) 1→2→1→2... 또는 1개 남으면 그것만 계속
                selected = 직전에 안간 접시 중 가장 가까운 것

                # Start → Bowl → Start 이동
                robot.Movej_Cmd(START_JOINTS, v=V_MOVE, r=0)   # Start로
                robot.Movej_P_Cmd(target_pose, v=V_MOVE, r=0)  # Bowl 위로
                robot.Movej_Cmd(START_JOINTS, v=V_MOVE, r=0)   # Start로 복귀
finally:
    pipeline.stop()
    robot.Arm_Socket_Close()
'''

# ─── 콘텐츠 블록 구성 ───
blocks_1 = [
    heading(1, '💻 Code Review — step7_twoplate_alwaysdetect.py'),
    para('다중 접시를 순차적으로 인식하고 로봇팔이 자동으로 왕복 이동하는 Pick & Place 테스트 코드입니다. '
         'YOLO 세그멘테이션 모델로 접시를 인식하고, Eye-to-Hand 캘리브레이션 행렬로 카메라 좌표를 로봇 좌표로 변환합니다.'),
    divider(),
    heading(2, '📄 전체 코드'),
    para('※ 가독성을 위해 설명 주석이 추가된 버전입니다. 실제 코드는 calibration/step7_twoplate_alwaysdetect.py 참고'),
    code_block(code_part1),
    code_block(code_part2),
    code_block(code_part3),
    code_block(code_part4),
    code_block(code_part5),
    code_block(code_part6),
]

blocks_2 = [
    divider(),
    heading(2, '📚 코드 상세 설명'),
    heading(3, '1. 전체 동작 흐름'),
    para('프로그램을 실행하면 카메라 화면이 열리고 아래 순서로 진행합니다:'),
    bullet('s키 → 로봇 현재 위치를 Start Point로 저장'),
    bullet('g키 → 연속 루프 시작 (이후 자동으로 반복)'),
    bullet('루프 안에서: 카메라로 접시 인식 → 가장 가까운 접시부터 이동 → Start 복귀 → 반복'),
    bullet('x키 → 루프 중지 / q키 → 프로그램 종료'),
    divider(),
    heading(3, '2. 캘리브레이션 행렬 (calibration_matrix.npy)'),
    para('카메라와 로봇이 서로 다른 좌표계를 사용하기 때문에 변환이 필요합니다.'),
    bullet('카메라 좌표: 카메라 렌즈 기준 (X=좌우, Y=상하, Z=앞뒤, 단위 mm)'),
    bullet('로봇 좌표: 로봇 베이스 기준 (X=앞뒤, Y=좌우, Z=높이, 단위 m)'),
    bullet('step3_calibration.py에서 7개 이상의 포인트를 수집해 변환 행렬을 계산 → npy 파일로 저장'),
    bullet('cam_to_robot() 함수: 카메라 XYZ → 행렬 곱 → 로봇 XYZ + 오프셋 보정'),
    divider(),
    heading(3, '3. 접시 중심 검출 (detect_bowl_center 함수)'),
    para('카메라가 55° 기울어져 있어 접시가 타원형으로 보입니다. 4단계로 중심을 계산합니다:'),
    bullet('Step 1: YOLO가 출력한 마스크(픽셀 영역)에서 윤곽선 추출'),
    bullet('Step 2: 윤곽선에 타원 피팅(fitEllipse) → 타원의 중심 픽셀 좌표 계산'),
    bullet('Step 3: 접시 가장자리(rim) 윤곽점 40개 샘플링 → depth 측정 → 25퍼센타일 값 사용 (접시 윗면 depth)'),
    bullet('Step 4: 픽셀+depth → rs2_deproject_pixel_to_point → 3D 카메라 좌표'),
    bullet('Step 5: 55° 기울기로 인한 편향 보정 + 고정 오프셋(X=-20, Y=+25mm) 적용'),
]

blocks_3 = [
    divider(),
    heading(3, '4. 연속 루프 구조 (g키)'),
    para('g키를 누르면 while running 루프가 시작되고 x키를 누를 때까지 반복됩니다.'),
    bullet('매 사이클마다 카메라로 접시를 새로 인식 (접시가 제거되면 자동 감지)'),
    bullet('직전에 방문한 접시 위치를 last_visited_rob_xyz에 저장'),
    bullet('다음 사이클: 직전 방문 접시와 가장 가까운 bowl을 "방금 간 곳"으로 판단 → 제외'),
    bullet('남은 접시 중 Start Point와 가장 가까운 것을 선택 → 교대 방문 (1→2→1→2...)'),
    bullet('접시가 1개만 남으면: 같은 접시라도 계속 이동'),
    divider(),
    heading(3, '5. 대기 루프 (접시 없을 때)'),
    para('접시가 모두 제거되면 로봇은 멈추고 카메라는 계속 감시합니다:'),
    bullet('화면에 "WAITING FOR BOWLS..." 표시'),
    bullet('500ms마다 YOLO로 재인식 시도'),
    bullet('접시가 다시 감지되면 last_visited_rob_xyz를 초기화하고 루프 재개'),
    bullet('대기 중에도 x키로 중지 가능'),
    divider(),
    heading(3, '6. 로봇 이동 명령'),
    para('로봇팔 이동에 두 가지 명령을 조합해서 사용합니다:'),
    bullet('Movej_Cmd(관절각, v=속도, r=0): 관절각으로 이동 → Start Point 복귀에 사용 (안정적)'),
    bullet('Movej_P_Cmd(XYZ+자세, v=속도, r=0): 직교좌표로 이동 → 접시 위치로 이동에 사용'),
    bullet('SAFE_HEIGHT=200mm: 접시 바로 위가 아닌 200mm 위로 이동 (충돌 방지)'),
    bullet('MIN_Z=80mm: 80mm 이하로는 이동 금지 (바닥 충돌 방지)'),
    bullet('이동 실패(작업 범위 초과) 시: Start 복귀 후 다음 사이클 계속 진행 (루프 종료 안 함)'),
    divider(),
    heading(3, '7. 오프셋 보정값 조정 방법'),
    para('카메라 위치가 바뀌면 오프셋을 다시 조정해야 합니다:'),
    bullet('step4_verify.py 또는 step5_test.py로 접시 위로 이동 테스트'),
    bullet('실제 로봇 TCP 위치와 접시 중심의 차이를 측정'),
    bullet('차이만큼 OFFSET_X_MM, OFFSET_Y_MM 값을 반대 방향으로 수정'),
    bullet('예: 로봇이 접시 중심보다 Y+ 방향으로 25mm 치우침 → OFFSET_Y_MM = +25'),
]

# ─── step7 페이지에 블록 업로드 ───
print('\n블록 추가 중 (1/3)...')
add_blocks(step7_id, blocks_1)
print('블록 추가 중 (2/3)...')
add_blocks(step7_id, blocks_2)
print('블록 추가 중 (3/3)...')
add_blocks(step7_id, blocks_3)

print('\n✅ 완료!')
print(f'Samsung Wellstory 프로젝트 진행 일지')
print(f'  └── 💻 Code Review')
print(f'       └── 🤖 step7_twoplate_alwaysdetect.py')
