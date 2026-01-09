#!/usr/bin/env python3
"""
UR 로봇 + OnRobot 그리퍼 통합 제어
하나의 코드에서 모션과 그리퍼 동시 제어
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from onrobot_gripper_driver.srv import Grip, Release
import time

class URWithGripper(Node):
    def __init__(self):
        super().__init__('ur_with_gripper')
        
        # UR Robot Action Client
        self.robot_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # OnRobot Gripper Service Clients
        self.grip_client = self.create_client(
            Grip, 
            '/onrobot_gripper_node/grip'
        )
        self.release_client = self.create_client(
            Release, 
            '/onrobot_gripper_node/release'
        )
        
        # Joint State Subscriber
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        self.current_joints = None
        
        self.get_logger().info('UR + Gripper 통합 제어 준비 완료')
        
        # Wait for services
        self.get_logger().info('Waiting for services...')
        self.grip_client.wait_for_service(timeout_sec=5.0)
        self.release_client.wait_for_service(timeout_sec=5.0)
        self.robot_client.wait_for_server(timeout_sec=5.0)
        self.get_logger().info('All services ready!')
    
    def joint_callback(self, msg):
        """현재 로봇 관절 각도"""
        self.current_joints = list(msg.position)
    
    def move_robot(self, target_joints, duration=3.0):
        """UR 로봇 이동"""
        goal = FollowJointTrajectory.Goal()
        
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = target_joints
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        
        trajectory.points.append(point)
        goal.trajectory = trajectory
        
        self.get_logger().info(f'🤖 로봇 이동 중...')
        
        future = self.robot_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        return True
    
    def grip(self, width=40.0, force=60.0, speed=50.0, wait=True):
        """그리퍼 닫기"""
        request = Grip.Request()
        request.target_position = width
        request.force = force
        request.speed = speed
        request.wait_for_completion = wait
        
        self.get_logger().info(f'✋ 그립: {width}mm, {force}N')
        
        future = self.grip_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        result = future.result()
        if result.success:
            self.get_logger().info(f'  ✓ 그립 완료 (실제: {result.actual_position:.1f}mm)')
            return True
        else:
            self.get_logger().error(f'  ✗ 그립 실패: {result.message}')
            return False
    
    def release(self, wait=True):
        """그리퍼 열기"""
        request = Release.Request()
        request.wait_for_completion = wait
        
        self.get_logger().info('✋ 릴리즈')
        
        future = self.release_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        result = future.result()
        if result.success:
            self.get_logger().info('  ✓ 릴리즈 완료')
            return True
        else:
            self.get_logger().error(f'  ✗ 릴리즈 실패: {result.message}')
            return False
    
    def pick_and_place(self):
        """완전한 Pick & Place 시퀀스"""
        self.get_logger().info('='*50)
        self.get_logger().info('🚀 Pick & Place 시작!')
        self.get_logger().info('='*50)
        
        # 0. Home 위치
        home_joints = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        self.get_logger().info('[1/9] Home으로 이동')
        self.move_robot(home_joints, duration=3.0)
        time.sleep(0.5)
        
        # 1. 그리퍼 열기
        self.get_logger().info('[2/9] 그리퍼 열기')
        self.release(wait=True)
        time.sleep(0.5)
        
        # 2. Pick 접근 위치 (물체 위)
        pick_approach = [0.5, -1.2, 1.3, -1.67, -1.57, 0.0]
        self.get_logger().info('[3/9] Pick 접근 위치로 이동')
        self.move_robot(pick_approach, duration=2.0)
        time.sleep(0.5)
        
        # 3. Pick 위치 (하강)
        pick_position = [0.5, -1.0, 1.1, -1.67, -1.57, 0.0]
        self.get_logger().info('[4/9] Pick 위치로 하강')
        self.move_robot(pick_position, duration=1.5)
        time.sleep(0.5)
        
        # 4. 그립!
        self.get_logger().info('[5/9] 물체 그립')
        if not self.grip(width=40.0, force=60.0, speed=50.0, wait=True):
            self.get_logger().error('❌ 그립 실패! 중단합니다.')
            return False
        time.sleep(0.5)
        
        # 5. 상승 (들어올리기)
        self.get_logger().info('[6/9] 물체 들어올리기')
        self.move_robot(pick_approach, duration=1.5)
        time.sleep(0.5)
        
        # 6. Place 위치로 이동
        place_approach = [-0.5, -1.2, 1.3, -1.67, -1.57, 0.0]
        self.get_logger().info('[7/9] Place 위치로 이동')
        self.move_robot(place_approach, duration=3.0)
        time.sleep(0.5)
        
        # 7. Place 위치 (하강)
        place_position = [-0.5, -1.0, 1.1, -1.67, -1.57, 0.0]
        self.get_logger().info('[8/9] Place 위치로 하강')
        self.move_robot(place_position, duration=1.5)
        time.sleep(0.5)
        
        # 8. 릴리즈!
        self.get_logger().info('[9/9] 물체 놓기')
        if not self.release(wait=True):
            self.get_logger().error('❌ 릴리즈 실패!')
            return False
        time.sleep(0.5)
        
        # 9. Home 복귀
        self.get_logger().info('[완료] Home으로 복귀')
        self.move_robot(home_joints, duration=3.0)
        
        self.get_logger().info('='*50)
        self.get_logger().info('✅ Pick & Place 성공!')
        self.get_logger().info('='*50)
        return True
    
    def demo_sequence(self):
        """데모 시퀀스: 여러 위치에서 그립 테스트"""
        self.get_logger().info('🎬 데모 시퀀스 시작')
        
        positions = [
            ([0.0, -1.57, 1.57, -1.57, -1.57, 0.0], 'Home'),
            ([0.5, -1.2, 1.3, -1.67, -1.57, 0.0], 'Position 1'),
            ([-0.5, -1.2, 1.3, -1.67, -1.57, 0.0], 'Position 2'),
        ]
        
        for joints, name in positions:
            self.get_logger().info(f'→ {name}으로 이동')
            self.move_robot(joints, duration=2.0)
            time.sleep(1.0)
            
            self.get_logger().info(f'  그립 테스트')
            self.grip(width=50.0, force=40.0, speed=50.0, wait=True)
            time.sleep(1.0)
            
            self.get_logger().info(f'  릴리즈')
            self.release(wait=True)
            time.sleep(1.0)
        
        self.get_logger().info('✅ 데모 완료!')

def main(args=None):
    rclpy.init(args=args)
    
    node = URWithGripper()
    
    try:
        # 사용자 선택
        print("\n" + "="*50)
        print("UR + OnRobot 그리퍼 통합 제어")
        print("="*50)
        print("1. Pick & Place 실행")
        print("2. 데모 시퀀스 실행")
        print("3. 종료")
        choice = input("선택 (1-3): ").strip()
        
        if choice == '1':
            node.pick_and_place()
        elif choice == '2':
            node.demo_sequence()
        else:
            print("종료합니다.")
    
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
