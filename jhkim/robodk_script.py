# ======================================================================
# 파일: robodk_script.py (동적 좌표 적용 버전)
# ======================================================================
# 역할: 1. 'coordinates.json' 파일에서 동적 Pick/Place 좌표를 읽어옵니다.
#      2. 읽어온 좌표를 바탕으로 RoboDK 내에서 로봇 시뮬레이션을 수행합니다.
#      3. 총 동작 시간을 측정하여 표준 출력으로 내보냅니다.
# ----------------------------------------------------------------------

from robolink import *
from robodk import *
import robodk.robomath as robolink_math
import json  # <--- json 라이브러리 추가
import os
import time

# --- 좌표 로딩 함수 추가 ---
def load_coordinates_from_file(filename="coordinates.json"):
    """ JSON 파일에서 시작/종료 좌표를 로드합니다. """
    if not os.path.exists(filename):
        raise FileNotFoundError(f"좌표 파일 '{filename}'을 찾을 수 없습니다.")
    with open(filename, 'r') as f:
        coords = json.load(f)
    print(f"좌표 로딩 완료: Start={coords['start_pos']}, End={coords['end_pos']}")
    return coords['start_pos'], coords['end_pos']

# --------------------------------------------------------------------------------
# 1. 연결 및 초기화
# --------------------------------------------------------------------------------
RDK = Robolink()

robot = RDK.Item('Fanuc LR Mate 200iD', ITEM_TYPE_ROBOT)
tool = RDK.Item('RobotiQ 2F-85 Gripper (Open)', ITEM_TYPE_TOOL)
robot_base = RDK.Item('Fanuc LR Mate 200iD Base')

if not robot.Valid() or not tool.Valid() or not robot_base.Valid():
    raise Exception("RoboDK 스테이션에서 로봇, 도구, 또는 베이스를 찾을 수 없습니다.")

robot.setTool(tool)
print("초기화 완료: 로봇, 도구, 객체를 성공적으로 불러왔습니다.")

# #속도 및 가속도 설정 추가
# robot.setSpeed(200)
# robot.setAcceleration(1000)

# --------------------------------------------------------------------------------
# 2. 변수 및 목표 정의 (★★핵심 수정 부분★★)
# --------------------------------------------------------------------------------

# --- 상수 정의 ---
HOME_JOINTS = [0, 0, 0, 0, 0, 0]
CUBE_HEIGHT = 100
SAFETY_APPROACH_DISTANCE = 100
GRIPPER_DO_ID = 0

# --- 동적 좌표 로딩 ---
start_pos_vec, end_pos_vec = load_coordinates_from_file()

# --- 목표 좌표 계산 ---
tool_orientation = roty(180 * pi / 180)

# 1. 월드 좌표계 기준 목표 포즈 계산 (JSON 파일의 좌표 사용)
# Pick 목표: BVH에서 분석된 start_pos_vec 좌표 사용
pose_pick_target_abs = transl(start_pos_vec[:3]) * transl(0, 0, CUBE_HEIGHT / 2) * tool_orientation

# Place 목표: BVH에서 분석된 end_pos_vec 좌표 사용
pose_place_target_abs = transl(end_pos_vec[:3]) * transl(0, 0, CUBE_HEIGHT / 2) * tool_orientation

# 2. 로봇 베이스 좌표계 기준 목표 포즈로 변환
pose_robot_base_abs = robot_base.Pose()
pose_pick_target_robot = invH(pose_robot_base_abs) * pose_pick_target_abs
pose_place_target_robot = invH(pose_robot_base_abs) * pose_place_target_abs

# 3. 접근(Approach) 및 후퇴(Retreat) 포즈 계산
approach_offset = transl(0, 0, -SAFETY_APPROACH_DISTANCE)
pose_pick_approach = pose_pick_target_robot * approach_offset
pose_place_approach = pose_place_target_robot * approach_offset

print("목표 좌표 계산 완료.")
print(f"Pick 목표 (로봇 기준): {pose_pick_target_robot.Pos()}")
print(f"Place 목표 (로봇 기준): {pose_place_target_robot.Pos()}")

# --------------------------------------------------------------------------------
# 3. 프로그램 실행 (이하 로직은 동일)
# --------------------------------------------------------------------------------
print("프로그램을 시작합니다...")
start_time = time.time()

robot.setDO(GRIPPER_DO_ID, 0)
pause(0.5)
robot.MoveJ(HOME_JOINTS)

# --- Pick Sequence ---
robot.MoveJ(pose_pick_approach)
robot.MoveL(pose_pick_target_robot)
robot.setDO(GRIPPER_DO_ID, 1)
pause(0.5)
tool.AttachClosest()
pause(0.5)
robot.MoveL(pose_pick_approach)

# --- Place Sequence ---
robot.MoveJ(pose_place_approach)
robot.MoveL(pose_place_target_robot)
robot.setDO(GRIPPER_DO_ID, 0)
pause(0.5)
tool.DetachAll()
pause(0.5)
robot.MoveL(pose_place_approach)

robot.MoveJ(HOME_JOINTS)

end_time = time.time()
elapsed_time = end_time - start_time

print("프로그램 실행이 성공적으로 완료되었습니다.")
print(f"Total Execution Time: {elapsed_time} seconds")