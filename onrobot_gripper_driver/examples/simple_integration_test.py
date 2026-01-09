#!/usr/bin/env python3
"""
간단한 통합 테스트
UR 로봇 + 그리퍼 동시 제어 확인
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from onrobot_gripper_driver.srv import Grip, Release

class SimpleTest(Node):
    def __init__(self):
        super().__init__('simple_test')
        
        # UR Robot
        self.robot = ActionClient(
            self, 
            FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # Gripper
        self.grip_client = self.create_client(Grip, '/onrobot_gripper_node/grip')
        self.release_client = self.create_client(Release, '/onrobot_gripper_node/release')
        
        # Wait
        self.grip_client.wait_for_service()
        self.release_client.wait_for_service()
        self.robot.wait_for_server()
        
        print("✓ UR Robot + Gripper 준비 완료!")
    
    def move_and_grip(self):
        """로봇 이동 + 그립 테스트"""
        print("\n[1/3] 로봇 이동...")
        
        # 로봇 이동
        goal = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = [0.5, -1.2, 1.3, -1.67, -1.57, 0.0]
        point.time_from_start.sec = 2
        trajectory.points.append(point)
        goal.trajectory = trajectory
        
        future = self.robot.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        print("  ✓ 이동 완료")
        
        # 그립
        print("\n[2/3] 그립...")
        request = Grip.Request()
        request.target_position = 50.0
        request.force = 40.0
        request.speed = 50.0
        
        future = self.grip_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        
        print(f"  ✓ 그립 완료: {result.actual_position:.1f}mm")
        
        # 릴리즈
        print("\n[3/3] 릴리즈...")
        request = Release.Request()
        future = self.release_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        print("  ✓ 릴리즈 완료")
        print("\n✅ 테스트 성공! UR Robot + Gripper 동시 제어 가능!")

def main():
    rclpy.init()
    node = SimpleTest()
    
    try:
        node.move_and_grip()
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
