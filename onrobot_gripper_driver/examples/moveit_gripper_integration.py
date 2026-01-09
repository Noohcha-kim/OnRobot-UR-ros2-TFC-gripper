#!/usr/bin/env python3
"""
MoveIt2 + OnRobot 그리퍼 통합
Cartesian 좌표로 로봇 제어 + 그리퍼
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import PoseStamped
from onrobot_gripper_driver.srv import Grip, Release

# MoveIt2 Python API (설치 필요)
try:
    from moveit.planning import MoveItPy
    from moveit.core.robot_state import RobotState
    MOVEIT_AVAILABLE = True
except ImportError:
    MOVEIT_AVAILABLE = False
    print("⚠️  MoveIt2 Python API가 설치되지 않았습니다.")
    print("설치: sudo apt install ros-humble-moveit-py")

class MoveItWithGripper(Node):
    def __init__(self):
        super().__init__('moveit_with_gripper')
        
        if not MOVEIT_AVAILABLE:
            raise RuntimeError("MoveIt2 Python API 필요")
        
        # MoveIt2 초기화
        self.moveit = MoveItPy(node_name="moveit_py")
        self.ur_arm = self.moveit.get_planning_component("ur_manipulator")
        
        # Gripper clients
        self.grip_client = self.create_client(Grip, '/onrobot_gripper_node/grip')
        self.release_client = self.create_client(Release, '/onrobot_gripper_node/release')
        
        self.grip_client.wait_for_service()
        self.release_client.wait_for_service()
        
        self.get_logger().info('MoveIt2 + Gripper 준비 완료!')
    
    def move_to_pose(self, x, y, z, roll=0.0, pitch=3.14, yaw=0.0):
        """Cartesian 좌표로 이동"""
        self.get_logger().info(f'Moving to: x={x}, y={y}, z={z}')
        
        # Pose 설정
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        
        # Orientation (quaternion으로 변환 필요)
        from scipy.spatial.transform import Rotation as R
        r = R.from_euler('xyz', [roll, pitch, yaw])
        quat = r.as_quat()
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        
        # Plan
        self.ur_arm.set_start_state_to_current_state()
        self.ur_arm.set_goal_state(pose_stamped_msg=pose, pose_link="tool0")
        
        plan_result = self.ur_arm.plan()
        
        if plan_result:
            self.get_logger().info('Plan 성공, 실행 중...')
            self.moveit.execute(plan_result.trajectory)
            return True
        else:
            self.get_logger().error('Planning 실패!')
            return False
    
    def grip(self, width=40.0, force=60.0):
        """그리퍼 닫기"""
        request = Grip.Request()
        request.target_position = width
        request.force = force
        request.speed = 50.0
        request.wait_for_completion = True
        
        future = self.grip_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success
    
    def release(self):
        """그리퍼 열기"""
        request = Release.Request()
        request.wait_for_completion = True
        
        future = self.release_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success
    
    def pick_and_place_cartesian(self):
        """Cartesian 좌표로 Pick & Place"""
        self.get_logger().info('🚀 Cartesian Pick & Place 시작')
        
        # Pick 접근
        self.get_logger().info('[1/5] Pick 접근')
        self.move_to_pose(x=0.4, y=0.2, z=0.4)
        
        # Pick 위치
        self.get_logger().info('[2/5] Pick 위치 하강')
        self.move_to_pose(x=0.4, y=0.2, z=0.2)
        
        # 그립
        self.get_logger().info('[3/5] 그립')
        self.grip(width=40.0, force=60.0)
        
        # 들어올리기
        self.get_logger().info('[4/5] 들어올리기')
        self.move_to_pose(x=0.4, y=0.2, z=0.4)
        
        # Place 위치
        self.get_logger().info('[5/5] Place')
        self.move_to_pose(x=0.4, y=-0.2, z=0.2)
        
        # 릴리즈
        self.release()
        
        self.get_logger().info('✅ 완료!')

def main():
    rclpy.init()
    
    if not MOVEIT_AVAILABLE:
        print("\n❌ MoveIt2 Python API를 먼저 설치하세요:")
        print("   sudo apt install ros-humble-moveit-py")
        return
    
    node = MoveItWithGripper()
    
    try:
        node.pick_and_place_cartesian()
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
