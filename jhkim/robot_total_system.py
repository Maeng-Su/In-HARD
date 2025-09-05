# ======================================================================
# 파일: robot_total_system.py 코드 통합(좌표값 추출 + 스크립트 + RoboDK 실행)
# ======================================================================
import os
import json
import re
import subprocess
import warnings
import time
import locale
import numpy as np
import pandas as pd
from pymo.parsers import BVHParser, MocapData
from pymo.preprocessing import MocapParameterizer
from pandas.errors import PerformanceWarning
import sys
import traceback

# --- ⚙️ 설정 변수 ---
warnings.filterwarnings("ignore", category=PerformanceWarning)
ROBODK_PATH = "C:/RoboDK/bin/RoboDK.exe"
STATION_PATH = "C:/gui_test.rdk"
# ROBOT_SCRIPT_PATH = "generated_robodk_script.py" #고정 좌표값
ROBOT_SCRIPT_PATH = "robodk_script.py" #동적 좌표값
BVH_FOLDER_PATH = "bvh"
MAPPING_CSV_PATH = "data/InHARD.csv"
COORDS_FILE = "coordinates.json"
#좌표계 변환 관련 설정(scale + offset)
SCALE = 1.0
X_OFFSET = 580.0
Y_OFFSET = -5.0
Z_OFFSET = 300.0

# ======================================================================
# 🔬 BVH 분석 함수 (성찬님 코드 그대로 반영함)
# ======================================================================

# 각 신체 부위의 좌표 컬럼을 전역 변수로 정의 (기존 코드와 동일)
hips_cols = ['Hips_Xposition', 'Hips_Yposition', 'Hips_Zposition']
left_hand_cols = ['LeftHand_Xposition', 'LeftHand_Yposition', 'LeftHand_Zposition']
right_hand_cols = ['RightHand_Xposition', 'RightHand_Yposition', 'RightHand_Zposition']

def slicing_bvh(start_bvh_frame, end_bvh_frame, original_data):
    # 3. 원본 데이터에서 원하는 범위만큼 smoothed_df을 슬라이싱합니다.
    sliced_values = original_data.values.iloc[start_bvh_frame:end_bvh_frame].copy()

    # 4. 슬라이싱된 smoothed_df과 기존 스켈레톤 정보로 새로운 MocapData 객체를 생성합니다.
    # 이렇게 하면 원하는 부분만 담긴 작은 모션 클립이 만들어집니다.
    sliced_data = MocapData()
    sliced_data.skeleton = original_data.skeleton
    sliced_data.values = sliced_values
    sliced_data.framerate = original_data.framerate
    sliced_data.channel_names = original_data.channel_names
    sliced_data.root_name = original_data.root_name

    return sliced_data

def transform_world_position(sliced_data, fk_calculator):
    # 6. 잘라낸 MocapData 객체만 변환기에 전달하여 월드 좌표를 계산합니다.
    global_positions_sliced_data = fk_calculator.fit_transform([sliced_data])[0]

    return global_positions_sliced_data

def get_pick_place_object_positions(global_positions_sliced_data, smoothing_window=10):
    # 노이즈 감소: 대상 프레임 주변의 평균을 계산할 범위 설정
    smoothed_df = global_positions_sliced_data.values.rolling(  
        window=smoothing_window,
        min_periods=1, 
        center=True
        ).mean()

    left_hand_vectors = smoothed_df[left_hand_cols].values - smoothed_df[hips_cols].values
    right_hand_vectors = smoothed_df[right_hand_cols].values - smoothed_df[hips_cols].values

    # 3. 각 벡터의 유클리디안 거리(L2 Norm)를 계산하고 새 컬럼에 추가합니다.
    #    np.linalg.norm 함수와 axis=1 옵션을 사용해 각 행(row)별로 거리를 계산합니다.
    smoothed_df['L_Hand_Hips_dist'] = np.linalg.norm(left_hand_vectors, axis=1)
    smoothed_df['R_Hand_Hips_dist'] = np.linalg.norm(right_hand_vectors, axis=1)

    # 1. 거리 계산에 사용된 두 컬럼만 선택합니다.
    dist_df = smoothed_df[['L_Hand_Hips_dist', 'R_Hand_Hips_dist']]
    # 2. 데이터를 stack()을 이용해 긴 형태로 변환합니다.
    stacked_dists = dist_df.stack()

    # 3. idxmax()를 사용해 최댓값의 인덱스(행, 컬럼명)를 찾습니다.
    row_index, hand_column = stacked_dists.idxmax()

    if 'L_Hand' in hand_column:
        cols_to_get = left_hand_cols
    else: # '오른손'인 경우
        cols_to_get = right_hand_cols

    # 2. .loc을 사용해 해당 행과 열의 좌표 값을 정확히 가져옵니다.
    start_position = smoothed_df.loc[row_index, cols_to_get].values

    end_position = smoothed_df.iloc[-1, :][cols_to_get].values
        
    return start_position, end_position

def get_pick_place_position(start_bvh_frame, end_bvh_frame, parsed_data):
    # start_bvh_frame, end_bvh_frame 구간의 parsed_data 데이터프레임
    sliced_data = slicing_bvh(start_bvh_frame, end_bvh_frame, parsed_data)

    # 월드 좌표계로 변환
    fk_calculator = MocapParameterizer('position') # 5. Forward Kinematics 계산기를 생성합니다.
    global_positions_sliced_data = transform_world_position(sliced_data, fk_calculator)

    # pick, place 좌표 구한 후 리턴
    return get_pick_place_object_positions(global_positions_sliced_data, smoothing_window=10)

"""BVH 좌표를 RoboDK 좌표계로 변환합니다."""
def transform_coordinates(position):
    transformed_pos = [
        position[0] * SCALE + X_OFFSET,
        position[1] * SCALE + Y_OFFSET,
        position[2] * SCALE + Z_OFFSET,
        0, 0, 0  # Orientation (Rx, Ry, Rz)
    ]
    return transformed_pos

# ======================================================================
# 🤖 RoboDK 실행 및 결과 파싱 함수
# ======================================================================
def run_robodk_simulation(coordinates):
    
    with open(COORDS_FILE, 'w') as f:
        json.dump(coordinates, f)
    print(f"▶️ 좌표를 '{COORDS_FILE}' 파일에 저장했습니다.")
    if not os.path.exists(ROBODK_PATH):
        raise FileNotFoundError(f"RoboDK 실행 파일을 찾을 수 없습니다: {ROBODK_PATH}")
    if not os.path.exists(STATION_PATH):
        raise FileNotFoundError(f"RoboDK 스테이션 파일을 찾을 수 없습니다: {STATION_PATH}")
    print(f"🚀 RoboDK GUI 실행 및 스테이션 로딩을 시작합니다...")
    subprocess.Popen([ROBODK_PATH, STATION_PATH])
    print("   - 스테이션 로드를 위해 5초간 대기합니다...")
    time.sleep(5)
    print(f"▶️ 로봇 스크립트 '{ROBOT_SCRIPT_PATH}'를 실행합니다...")
    result = subprocess.run(
        [sys.executable, ROBOT_SCRIPT_PATH],
        check=True, capture_output=True, text=True, encoding=locale.getpreferredencoding()
    )
    print("✅ 로봇 스크립트 실행 완료.")
    output = result.stdout
    match = re.search(r"Total Execution Time: ([\d.]+) seconds", output)
    if match:
        execution_time = float(match.group(1))
        print(f"⏱️ 동작 시간 파싱 성공: {execution_time:.2f}초")
        return execution_time
    else:
        raise ValueError("스크립트 출력에서 'Total Execution Time'을 찾을 수 없습니다.")

# ======================================================================
# 🚀 메인 실행 블록 (전체 로직을 하나로 통합)
# ======================================================================
if __name__ == "__main__":
    file_id = input("▶️ 분석할 FILE_ID를 입력하고 Enter를 누르세요 (예: P01_R01): ")
    
    if not file_id:
        print("‼️ FILE_ID가 입력되지 않았습니다. 프로그램을 종료합니다.")
    else:
        try:
            # --- 1. 파일 경로 설정 및 확인 ---
            bvh_path = os.path.join(BVH_FOLDER_PATH, f"{file_id}.bvh")
            print(f"▶️ '{bvh_path}' 파일 분석을 시작합니다...")
            if not os.path.exists(bvh_path):
                raise FileNotFoundError(f"BVH 파일을 찾을 수 없습니다: {bvh_path}")
            if not os.path.exists(MAPPING_CSV_PATH):
                raise FileNotFoundError(f"매핑 파일을 찾을 수 없습니다: {MAPPING_CSV_PATH}")

            # --- 2. 데이터 로드 ---
            parser = BVHParser()
            parsed_data = parser.parse(bvh_path)
                        
            # --- 3. 매핑 데이터 필터링 ---
            mapping_df = pd.read_csv(MAPPING_CSV_PATH)
            cond = mapping_df['File'] == file_id
            cond2 = mapping_df['Meta_action_class_number'].isin([6, 7])
            filtered_df = mapping_df[cond & cond2]
            if filtered_df.empty:
                raise ValueError(f"'{MAPPING_CSV_PATH}'에서 '{file_id}'에 해당하는 Pick Action을 찾을 수 없습니다.")

            # 4. GT 시간 계산 및 시뮬레이션 시간 변수 초기화
            total_gt_time = filtered_df['Duration_sec'].sum() # 전체 GT 시간을 미리 합산
            total_simulation_time = 0.0 # 시뮬레이션 시간을 합산할 변수
            action_count = 0
            
            print(f"\n✅ 총 {len(filtered_df)}개의 Action을 발견했습니다. 분석을 시작합니다.")

            # 5. 발견된 모든 Action에 대해 반복 실행
            for index, action_row in filtered_df.iterrows():
                action_count += 1
                start_frame = int(action_row['Action_start_bvh_frame'])
                end_frame = int(action_row['Action_end_bvh_frame'])
                
                print(f"\n--- [{action_count}/{len(filtered_df)}] 번째 동작 분석 (프레임: {start_frame}~{end_frame}) ---")
                
                # 좌표 계산
                start_position, end_position = get_pick_place_position(start_frame, end_frame, parsed_data)
                
                # # 좌표 형식 변환 (좌표계 변환 로직은 아직 미적용)
                # start_coords = np.append(start_position, [0, 0, 0]).tolist()
                # end_coords = np.append(end_position, [0, 0, 0]).tolist()

                # 함수를 호출하여 좌표계 변환 적용
                start_coords = transform_coordinates(start_position)
                end_coords = transform_coordinates(end_position)

                print(f"   - Pick  (Start): {np.round(start_coords[:3], 2)}")
                print(f"   - Place (End)  : {np.round(end_coords[:3], 2)}")

                # RoboDK 실행 및 시간 측정
                coords_for_script = {'start_pos': start_coords, 'end_pos': end_coords}
                execution_time = run_robodk_simulation(coords_for_script)
                
                # 시뮬레이션 시간 합산
                total_simulation_time += execution_time

            # 6. 최종 결과 출력
            print("\n" + "="*50)
            print("🎉 모든 작업이 성공적으로 완료되었습니다!")
            print(f"   - 분석 대상: '{file_id}' 파일의 모든 6, 7번 Action")
            print(f"   - 처리된 총 동작 수: {action_count} 개")
            print("-" * 50)
            print(f"   - 👩‍💻 총 GT 시간 (사람 기준): {total_gt_time:.2f} 초")
            print(f"   - 🤖 총 시뮬레이션 시간 (로봇 기준): {total_simulation_time:.2f} 초")
            print("="*50)

        except Exception as e:
            print("\n" + "!"*50)
            print(f"‼️ 오류가 발생했습니다: {e}")
            print("\n--- 상세 오류 정보 (Traceback) ---")
            print(traceback.format_exc())
            print("------------------------------------")