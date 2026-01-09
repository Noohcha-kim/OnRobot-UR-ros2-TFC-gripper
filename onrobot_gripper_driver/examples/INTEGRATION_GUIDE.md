# UR Robot + OnRobot Gripper 통합 제어 가이드

## ✅ 네, 완전히 가능합니다!

UR 로봇 드라이버와 그리퍼 드라이버가 **별도로 실행**되지만,  
**하나의 코드에서 동시에 제어** 가능합니다! 🎯

---

## 🏗️ 아키텍처

```
Python/C++ 통합 코드
    ↓           ↓
  UR Robot   Gripper
  Driver     Driver
    ↓           ↓
  로봇 모션   그리퍼
```

**핵심:** ROS2의 **Action Client/Service Client**를 사용하면  
별도 노드들을 하나의 코드에서 제어 가능!

---

## 📦 준비물

### 1. 실행 중인 노드들

```bash
# Terminal 1: socat (백그라운드)
./start_socat.sh

# Terminal 2: UR Robot Driver
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=192.168.56.101

# Terminal 3: Gripper Driver
ros2 launch onrobot_gripper_driver onrobot_gripper.launch.py
```

### 2. Python 스크립트

3가지 예제 제공:
1. **simple_integration_test.py** - 기본 테스트
2. **ur_gripper_integration.py** - 완전한 Pick & Place
3. **moveit_gripper_integration.py** - MoveIt2 통합

---

## 🚀 방법 1: 간단한 테스트

### simple_integration_test.py

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from onrobot_gripper_driver.srv import Grip, Release

class SimpleTest:
    def __init__(self):
        # UR Robot
        self.robot = ActionClient(...)
        
        # Gripper
        self.grip_client = self.create_client(Grip, ...)
        self.release_client = self.create_client(Release, ...)
    
    def move_and_grip(self):
        # 1. 로봇 이동
        self.robot.send_goal_async(...)
        
        # 2. 그립
        self.grip_client.call_async(...)
        
        # 3. 릴리즈
        self.release_client.call_async(...)
```

### 실행

```bash
# Terminal 4: 통합 제어
python3 simple_integration_test.py
```

**출력:**
```
✓ UR Robot + Gripper 준비 완료!

[1/3] 로봇 이동...
  ✓ 이동 완료

[2/3] 그립...
  ✓ 그립 완료: 48.5mm

[3/3] 릴리즈...
  ✓ 릴리즈 완료

✅ 테스트 성공! UR Robot + Gripper 동시 제어 가능!
```

---

## 🎯 방법 2: 완전한 Pick & Place

### ur_gripper_integration.py

**9단계 Pick & Place 시퀀스:**

```python
def pick_and_place(self):
    # 1. Home
    self.move_robot(home_joints)
    
    # 2. 그리퍼 열기
    self.release()
    
    # 3. Pick 접근
    self.move_robot(pick_approach)
    
    # 4. Pick 하강
    self.move_robot(pick_position)
    
    # 5. 그립!
    self.grip(width=40.0, force=60.0)
    
    # 6. 들어올리기
    self.move_robot(pick_approach)
    
    # 7. Place 이동
    self.move_robot(place_position)
    
    # 8. 하강
    self.move_robot(place_position)
    
    # 9. 릴리즈!
    self.release()
```

### 실행

```bash
python3 ur_gripper_integration.py
```

**출력:**
```
==================================================
🚀 Pick & Place 시작!
==================================================
[1/9] Home으로 이동
🤖 로봇 이동 중...
[2/9] 그리퍼 열기
✋ 릴리즈
  ✓ 릴리즈 완료
[3/9] Pick 접근 위치로 이동
🤖 로봇 이동 중...
[4/9] Pick 위치로 하강
🤖 로봇 이동 중...
[5/9] 물체 그립
✋ 그립: 40.0mm, 60.0N
  ✓ 그립 완료 (실제: 38.5mm)
[6/9] 물체 들어올리기
🤖 로봇 이동 중...
[7/9] Place 위치로 이동
🤖 로봇 이동 중...
[8/9] Place 위치로 하강
🤖 로봇 이동 중...
[9/9] 물체 놓기
✋ 릴리즈
  ✓ 릴리즈 완료
[완료] Home으로 복귀
🤖 로봇 이동 중...
==================================================
✅ Pick & Place 성공!
==================================================
```

---

## 🎨 방법 3: MoveIt2 통합

### moveit_gripper_integration.py

**Cartesian 좌표로 제어:**

```python
def pick_and_place_cartesian(self):
    # Pick 접근
    self.move_to_pose(x=0.4, y=0.2, z=0.4)
    
    # Pick 하강
    self.move_to_pose(x=0.4, y=0.2, z=0.2)
    
    # 그립
    self.grip(width=40.0, force=60.0)
    
    # 들어올리기
    self.move_to_pose(x=0.4, y=0.2, z=0.4)
    
    # Place
    self.move_to_pose(x=0.4, y=-0.2, z=0.2)
    
    # 릴리즈
    self.release()
```

### 실행

```bash
# MoveIt2 설치 (필요시)
sudo apt install ros-humble-moveit-py

# 실행
python3 moveit_gripper_integration.py
```

---

## 📝 스크립트 설치

```bash
# 스크립트 다운로드 (제공된 파일)
cd ~
chmod +x ur_gripper_integration.py
chmod +x simple_integration_test.py
chmod +x moveit_gripper_integration.py
```

---

## 🔧 커스터마이징

### 관절 각도 수정

```python
# ur_gripper_integration.py 수정
pick_position = [0.5, -1.0, 1.1, -1.67, -1.57, 0.0]  # 여기 수정
```

### 그립 파라미터 수정

```python
self.grip(
    width=40.0,   # 목표 폭 (mm)
    force=60.0,   # 힘 (N)
    speed=50.0,   # 속도 (%)
    wait=True     # 완료까지 대기
)
```

### Cartesian 좌표 수정

```python
# moveit_gripper_integration.py 수정
self.move_to_pose(
    x=0.4,        # X 좌표 (m)
    y=0.2,        # Y 좌표 (m)
    z=0.3,        # Z 좌표 (m)
    roll=0.0,     # Roll (rad)
    pitch=3.14,   # Pitch (rad)
    yaw=0.0       # Yaw (rad)
)
```

---

## 🎓 핵심 개념

### 1. Action Client (로봇 모션)

```python
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory

robot_client = ActionClient(
    self,
    FollowJointTrajectory,
    '/scaled_joint_trajectory_controller/follow_joint_trajectory'
)

# 비동기 호출
future = robot_client.send_goal_async(goal)
rclpy.spin_until_future_complete(self, future)
```

### 2. Service Client (그리퍼)

```python
from onrobot_gripper_driver.srv import Grip, Release

grip_client = self.create_client(Grip, '/onrobot_gripper_node/grip')

# 비동기 호출
request = Grip.Request()
future = grip_client.call_async(request)
rclpy.spin_until_future_complete(self, future)
```

### 3. 동기화

```python
# 순차 실행
move_robot()        # 1. 로봇 이동 (완료까지 대기)
grip()              # 2. 그립 (완료까지 대기)
move_robot()        # 3. 다시 이동

# 병렬 실행 (고급)
future1 = move_robot_async()
future2 = grip_async()
rclpy.spin_until_future_complete(self, future1)
rclpy.spin_until_future_complete(self, future2)
```

---

## 🐛 트러블슈팅

### "No node found"

```bash
# 모든 노드 실행 확인
ros2 node list

# 예상 출력:
# /ur_robot_driver_node
# /onrobot_gripper_node
```

### "Service not available"

```bash
# 그리퍼 서비스 확인
ros2 service list | grep grip

# 예상 출력:
# /onrobot_gripper_node/grip
# /onrobot_gripper_node/release
```

### "Action server not available"

```bash
# UR 드라이버 Action 확인
ros2 action list

# 예상 출력:
# /scaled_joint_trajectory_controller/follow_joint_trajectory
```

---

## 📊 성능

### 응답 시간

- **로봇 이동:** 1-5초 (거리에 따라)
- **그립:** 0.2-1초
- **릴리즈:** 0.2-0.5초

### 정확도

- **위치:** ±0.5mm (그리퍼)
- **힘:** ±2N (그리퍼)

---

## 💡 추천 워크플로우

### 개발 단계

```bash
# 1. 각 노드 개별 테스트
ros2 service call /onrobot_gripper_node/grip ...

# 2. 간단한 통합 테스트
python3 simple_integration_test.py

# 3. 완전한 시퀀스 테스트
python3 ur_gripper_integration.py
```

### 프로덕션

```bash
# systemd로 socat 자동 시작
sudo systemctl start onrobot-tool-comm.service

# Launch 파일로 통합
ros2 launch my_package robot_with_gripper.launch.py
```

---

## 🚀 다음 단계

1. **비전 통합:** 카메라로 물체 위치 인식
2. **Force Control:** 힘 센서로 안전한 그립
3. **궤적 최적화:** 최단 경로 계산
4. **에러 복구:** 실패 시 자동 재시도

---

**완전히 가능합니다!** 🎉

질문이나 문제가 있으면 언제든지 물어보세요!
