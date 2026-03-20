"""
SAM auto-annotate로 sam_test 폴더 전체 라벨링
"""
from ultralytics.data.annotator import auto_annotate

sam_input  = r'C:\Users\ASUS\Desktop\datasets\videos\images'
sam_output = r'C:\Users\ASUS\Desktop\datasets\videos\labels'

print("SAM auto-annotate 전체 실행 중...")
auto_annotate(
    data=sam_input,
    det_model=r'C:\Users\ASUS\Desktop\samsung wellstory\runs\segment\bowl_seg_v36\weights\best.pt',
    sam_model='sam2.1_b.pt',
    output_dir=sam_output,
    device=0,
)

import os
labels = [f for f in os.listdir(sam_output) if f.endswith('.txt')]
print(f"\n완료! 생성된 라벨: {len(labels)}개")
