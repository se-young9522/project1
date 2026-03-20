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

# Samsung Wellstory 프로젝트 페이지 찾기
print('Samsung Wellstory 페이지 검색 중...')
children = api(f'https://api.notion.com/v1/blocks/{PROJECT_PAGE_ID}/children?page_size=100')
sw_page_id = None
for block in children.get('results', []):
    if block['type'] == 'child_page':
        title = block.get('child_page', {}).get('title', '')
        if 'Samsung' in title or 'Wellstory' in title or '삼성' in title:
            sw_page_id = block['id']
            break

if not sw_page_id:
    print('Samsung Wellstory 페이지를 찾을 수 없습니다.')
    sys.exit(1)
print(f'Samsung Wellstory 페이지 ID: {sw_page_id}')

# ========================================
# 3월 17일 (화)
# ========================================
print('\n=== 2026-03-17 (화) 페이지 생성 ===')
day17 = api('https://api.notion.com/v1/pages', {
    'parent': {'page_id': sw_page_id},
    'icon': {'type': 'emoji', 'emoji': '\U0001f4dd'},
    'properties': {
        'title': {'title': [{'text': {'content': '2026-03-17 (화)'}}]}
    },
}, method='POST')
day17_id = day17['id']

# 1. Eye-to-Hand 캘리브레이션 코드 개선
print('  1. 캘리브레이션 코드 개선 작성 중...')
api(f'https://api.notion.com/v1/blocks/{day17_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '1. Eye-to-Hand 캘리브레이션 코드 개선 (step3_calibration.py)'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '상태: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '완료'}, 'annotations': {'color': 'green', 'bold': True}},
            ]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': 'RealSense D435i 최적화'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '레이저 파워 최대화 (depth 신호 강화)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'depth 필터 튜닝: spatial(alpha=0.5, delta=20), temporal(alpha=0.4), hole_filling 추가'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '안정화 프레임 30→60 (D435i auto-exposure 수렴 대기)'}}]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': 'Depth 정확도 개선'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '클릭 시 5x5 영역 depth 중앙값(median) 사용 (단일 픽셀 노이즈 방지)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '최소 depth 임계값 0.1→0.2m (D435i 최소 측정거리 ~0.28m 고려)'}}]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '코드 리뷰: step3_calibration.py D435i 최적화'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': 'D435i의 Active IR Stereo 특성에 맞춰 레이저 파워를 최대로 설정하여 depth 신호를 강화. spatial/temporal/hole_filling 3단계 필터 파이프라인으로 depth 노이즈를 줄이고, 클릭 시 5x5 영역의 중앙값을 사용하여 개별 픽셀의 불안정한 depth 값을 보정. D435i의 최소 측정거리(~0.28m)를 고려하여 depth 임계값을 0.2m로 상향 조정.'}}]}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# 2. 빨간 마커 중심 자동 검출
print('  2. 마커 검출 기능 작성 중...')
api(f'https://api.notion.com/v1/blocks/{day17_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '2. 빨간 마커 중심 자동 검출 기능 추가'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '상태: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '완료'}, 'annotations': {'color': 'green', 'bold': True}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '엔드툴 볼트에 빨간색 마커(스티커) 부착'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'HSV 색상 기반 빨간 마커 검출 (H: 0~10 + 170~180, S/V: 50+)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '160x160 crop 영역에서 마커 무게중심(centroid) 자동 보정'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '모폴로지(open/close) + 윤곽선 원형도 필터링 (circularity > 0.5)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '검출 실패 시 클릭 위치 그대로 사용 (fallback)'}}]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '시행착오'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '초기: 흰색 마커(threshold 180) → 검출 실패 (스티커 너무 작고, 때로 인해 밝기 부족)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'threshold 150 + 최소면적 15로 조정해도 불안정'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '최종: 빨간색으로 변경 → HSV 기반 색상 검출로 안정적 검출 달성'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'crop 80x80 → 160x160 확대 (클릭 위치에 따라 결과 변동 문제 해결)'}}]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '코드 리뷰: 빨간 마커 검출 로직'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '클릭 주변 160x160 영역을 crop하여 HSV 색공간으로 변환. 빨간색은 H값이 0 근처와 170 근처에 분포하므로 두 범위의 마스크를 OR 결합. 모폴로지 연산으로 노이즈를 제거한 뒤 윤곽선을 찾고, 원형도(circularity) 0.5 이상인 영역 중 가장 큰 것의 무게중심(moments)을 마커 중심으로 사용. crop 내 좌표를 전체 이미지 좌표로 변환하여 보정된 위치에서 depth를 측정.'}}]}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# 3. 포인트 관리 기능 개선
print('  3. 포인트 관리 기능 작성 중...')
api(f'https://api.notion.com/v1/blocks/{day17_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '3. 캘리브레이션 포인트 관리 기능 개선'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '상태: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '완료'}, 'annotations': {'color': 'green', 'bold': True}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'z키: 번호 지정 삭제 기능 추가 (기존: 마지막 포인트만 삭제 가능)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '포인트 목록 전체 출력 → 번호 입력 → 해당 포인트 삭제'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'Enter=마지막 삭제(기존 동작 유지), 0=취소'}}]}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# 4. 캘리브레이션 테스트 결과
print('  4. 캘리브레이션 테스트 결과 작성 중...')
api(f'https://api.notion.com/v1/blocks/{day17_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '4. 캘리브레이션 테스트 및 검증'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '상태: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '진행 중 (재캘리브레이션 필요)'}, 'annotations': {'color': 'orange', 'bold': True}},
            ]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '테스트 결과'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '9포인트 수집 후 캘리브레이션 수행'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'Y축 -40mm 오차 → Y방향 포인트 분포가 좁았음이 원인'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'Z축 +160mm → SAFE_HEIGHT 200mm 포함 여부 확인 (접시 Z=46mm + 200mm = 246mm)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'TCP 좌표 기준 xyz 오차 5mm 이내 달성 (시차 효과로 카메라에서는 밀려 보임)'}}]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '정확도 기준 (목표: 3~5mm)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '우수: 1~3mm (정밀 조립) / 양호: 3~5mm (일반 pick & place) ← 우리 목표'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '보통: 5~10mm (큰 물체) / 불량: 10mm+ (재캘리브레이션 필요)'}}]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '카메라 고정 문제'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '현재 임시 고정 상태 → 흔들림이 캘리브레이션 오차에 영향'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '권장: L자 브라켓 + C클램프 (D435i 무게 72g, 1/4"-20 삼각대 마운트 활용)'}}]}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# 5. step4_verify.py 개선
print('  5. step4 개선 작성 중...')
api(f'https://api.notion.com/v1/blocks/{day17_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '5. step4_verify.py 접시 인식 정확도 개선'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '상태: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '완료'}, 'annotations': {'color': 'green', 'bold': True}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'YOLO bounding box 중심 → seg 마스크 무게중심(centroid)으로 변경'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'seg 마스크를 1280x720으로 리사이즈 후 cv2.moments로 무게중심 계산'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '카메라 각도에 관계없이 접시의 실제 중심을 더 정확하게 계산'}}]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '코드 리뷰: step4_verify.py seg 마스크 활용'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '기존: YOLO detection box의 (x1+x2)/2, (y1+y2)/2로 중심 계산 → 카메라 각도에 따라 실제 접시 중심과 차이 발생. 개선: bowl_seg_v53 모델의 segmentation 마스크를 추출하여 cv2.moments로 무게중심을 계산. 마스크의 모든 픽셀을 고려하므로 비스듬한 각도에서도 접시의 실제 중심에 더 가까운 좌표를 얻을 수 있음.'}}]}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# 6. 유사 프로젝트 조사
print('  6. 유사 프로젝트 조사 작성 중...')
api(f'https://api.notion.com/v1/blocks/{day17_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '6. 유사 프로젝트 조사 및 분석'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '상태: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '완료'}, 'annotations': {'color': 'green', 'bold': True}},
            ]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '참조 프로젝트 3건'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'KUKA iiwa + YOLOv4 (ROS): YOLO 인식 → 좌표 변환 → 로봇 이동 파이프라인 참조'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'PKUAILab Eye-to-Hand + D435i: cv2.calibrateHandEye(Tsai-Lenz) + 체스보드 → 우리 SVD 방식과 비교'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '4DoF 로봇팔 + OpenCV: 2D 캘리브레이션(cm_per_pixel) → 우리 3D 방식에는 적용 불가'}}]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '비교 결과'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'PKUAILab cali.py: Tsai-Lenz + 체스보드 vs 우리: SVD + depth 직접 3D → 체스보드 불필요'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '캘리브레이션은 위치(xyz)만 담당, 자세(rx,ry,rz)는 별도 고정값으로 분리'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'Pick & Place 자동화 루프 패턴 정리: 흡착→들어올리기→이동→내리기→배치→복귀'}}]}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# 향후 계획
print('  향후 계획 작성 중...')
api(f'https://api.notion.com/v1/blocks/{day17_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '향후 계획'}}]}},
        {'object': 'block', 'type': 'to_do', 'to_do': {
            'rich_text': [{'type': 'text', 'text': {'content': '카메라 고정 개선 (C클램프 + L브라켓)'}}],
            'checked': False}},
        {'object': 'block', 'type': 'to_do', 'to_do': {
            'rich_text': [{'type': 'text', 'text': {'content': '캘리브레이션 재수행 (10포인트, X/Y/Z 넓은 분포, 특히 Y방향)'}}],
            'checked': False}},
        {'object': 'block', 'type': 'to_do', 'to_do': {
            'rich_text': [{'type': 'text', 'text': {'content': 'step4 검증 (Y 오차 5mm 이내 목표)'}}],
            'checked': False}},
        {'object': 'block', 'type': 'to_do', 'to_do': {
            'rich_text': [{'type': 'text', 'text': {'content': '그리퍼 장착 후 실제 pick & place 테스트'}}],
            'checked': False}},
    ]
}, method='PATCH')

print('\n완료!')
print('프로젝트 진행 일지')
print('  └── Samsung Wellstory')
print('       └── 2026-03-17 (화)')
