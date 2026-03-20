"""
라벨 시각화 뷰어 (Polygon Segmentation)

사용법:
  python label_viewer.py                        # labels_v2 전체 보기
  python label_viewer.py --only-auto            # 자동 라벨만 보기 (수작업 제외)
  python label_viewer.py --start 50             # 50번째 이미지부터 시작

키보드 조작:
  → / d   다음 이미지
  ← / a   이전 이미지
  q        종료
  s        현재 이미지 저장 (review_output 폴더)
"""

import argparse
import os
import cv2
import numpy as np

# ─── 경로 설정 ─────────────────────────────────────────
#IMAGE_DIR   = r'C:\Users\ASUS\Desktop\datasets\images'
#LABEL_DIR   = r'C:\Users\ASUS\Desktop\datasets\labels'
IMAGE_DIR = r'C:\Users\ASUS\Desktop\datasets\videos\images'
LABEL_DIR   = r'C:\Users\ASUS\Desktop\datasets\videos\labels'
OUTPUT_DIR  = r'C:\Users\ASUS\Desktop\datasets\review_output'
# ────────────────────────────────────────────────────────

POLYGON_COLOR = (0, 255, 0)    # 폴리곤 선: 초록
FILL_ALPHA    = 0.25           # 채우기 투명도
FONT          = cv2.FONT_HERSHEY_SIMPLEX


def load_label(label_path, img_w, img_h):
    """YOLO polygon 형식 → 픽셀 좌표 리스트 반환"""
    polygons = []
    with open(label_path, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) < 5:
                continue
            # cls x1 y1 x2 y2 ... (정규화 좌표)
            coords = list(map(float, parts[1:]))
            pts = np.array(coords).reshape(-1, 2)
            pts[:, 0] *= img_w
            pts[:, 1] *= img_h
            polygons.append(pts.astype(np.int32))
    return polygons


def draw_polygons(img, polygons):
    """이미지 위에 폴리곤 그리기 (반투명 채우기 + 외곽선)"""
    overlay = img.copy()
    for pts in polygons:
        cv2.fillPoly(overlay, [pts], (0, 200, 0))
    img = cv2.addWeighted(overlay, FILL_ALPHA, img, 1 - FILL_ALPHA, 0)
    for pts in polygons:
        cv2.polylines(img, [pts], isClosed=True, color=POLYGON_COLOR, thickness=2)
        # 꼭짓점 표시
        for pt in pts:
            cv2.circle(img, tuple(pt), 3, (0, 255, 255), -1)
    return img


def make_info_bar(img, filename, idx, total, n_polygons):
    """상단 정보 바 추가"""
    bar_h = 36
    bar = np.zeros((bar_h, img.shape[1], 3), dtype=np.uint8)
    text = f"[{idx+1}/{total}] {filename}  |  폴리곤: {n_polygons}개  |  ←→:이동  s:저장  q:종료"
    cv2.putText(bar, text, (8, 24), FONT, 0.55, (200, 200, 200), 1, cv2.LINE_AA)
    return np.vstack([bar, img])


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--only-auto', action='store_true',
                        help='수작업 라벨 목록을 제외하고 자동 라벨만 확인')
    parser.add_argument('--manual-list', type=str, default='',
                        help='수작업 라벨 파일명 목록 텍스트 파일 (--only-auto 옵션과 함께 사용)')
    parser.add_argument('--start', type=int, default=0,
                        help='시작 인덱스')
    return parser.parse_args()


def main():
    args = parse_args()
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # 라벨이 있는 이미지 목록
    label_files = sorted(f for f in os.listdir(LABEL_DIR) if f.endswith('.txt'))

    # --only-auto: 수작업 목록 제외
    if args.only_auto and args.manual_list and os.path.exists(args.manual_list):
        with open(args.manual_list) as f:
            manual = set(line.strip() for line in f)
        label_files = [f for f in label_files if f not in manual]
        print(f"자동 라벨만 표시: {len(label_files)}장")
    else:
        print(f"라벨 총 {len(label_files)}장")

    if not label_files:
        print("표시할 라벨이 없습니다.")
        return

    idx = max(0, min(args.start, len(label_files) - 1))

    while True:
        label_fname = label_files[idx]
        img_fname   = label_fname.replace('.txt', '.jpg')
        img_path    = os.path.join(IMAGE_DIR, img_fname)
        label_path  = os.path.join(LABEL_DIR, label_fname)

        # 이미지 로드
        img = cv2.imread(img_path)
        if img is None:
            # 이미지 없으면 빈 화면
            img = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(img, f"이미지 없음: {img_fname}", (20, 240),
                        FONT, 0.7, (0, 0, 255), 2)
            polygons = []
        else:
            h, w = img.shape[:2]
            polygons = load_label(label_path, w, h)
            img = draw_polygons(img, polygons)

        # 정보 바
        img = make_info_bar(img, img_fname, idx, len(label_files), len(polygons))

        cv2.imshow('Label Viewer', img)
        key = cv2.waitKeyEx(0)

        if key in (ord('q'), 27):              # q / ESC → 종료
            break
        elif key in (ord('d'), 2555904):       # → / d → 다음
            idx = (idx + 1) % len(label_files)
        elif key in (ord('a'), 2424832):       # ← / a → 이전
            idx = (idx - 1) % len(label_files)
        elif key == ord('s'):                  # s → 저장
            out_path = os.path.join(OUTPUT_DIR, img_fname)
            cv2.imwrite(out_path, img)
            print(f"저장: {out_path}")

    cv2.destroyAllWindows()
    print("종료")


if __name__ == '__main__':
    main()
