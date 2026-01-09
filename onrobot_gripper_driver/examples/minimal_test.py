#!/usr/bin/env python3
"""
최소 예제: UR 로봇 + 그리퍼 동시 제어
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from onrobot_gripper_driver.srv import Grip, Release

class MinimalExample(Node):
    def __init__(self):
        super().__init__('minimal_example')
        
        # UR 로봇 클라이언트
        self.robot = ActionClient(
            self, 
            FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # 그리퍼 클라이언트
        self.grip = self.create_client(Grip, '/onrobot_gripper_node/grip')
        self.release = self.create_client(Release, '/onrobot_gripper_node/release')
        
        print("대기 중...")
        self.robot.wait_for_server()
        self.grip.wait_for_service()
        self.release.wait_for_service()
        print("✓ 준비 완료!\n")
    
    def move(self, joints):
        """로봇 이동"""
        print(f"🤖 로봇 이동: {joints}")
        
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 
                           'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        point = JointTrajectoryPoint()
        point.positions = joints
        point.time_from_start.sec = 2
        traj.points = [point]
        goal.trajectory = traj
        
        future = self.robot.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        result = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result)
        print("  ✓ 이동 완료\n")
    
    def do_grip(self):
        """그립"""
        print("✋ 그립!")
        req = Grip.Request()
        req.target_position = 40.0
        req.force = 60.0
        req.speed = 50.0
        future = self.grip.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        print(f"  ✓ 그립 완료: {future.result().actual_position:.1f}mm\n")
    
    def do_release(self):
        """릴리즈"""
        print("✋ 릴리즈!")
        req = Release.Request()
        future = self.release.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        print("  ✓ 릴리즈 완료\n")

def main():
    rclpy.init()
    node = MinimalExample()
    
    print("="*40)
    print("로봇 + 그리퍼 동시 제어 테스트")
    print("="*40 + "\n")
    
    # 1. 위치 A로 이동
    node.move([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])
    
    # 2. 그립
    node.do_grip()
    
    # 3. 위치 B로 이동
    node.move([0.5, -1.2, 1.3, -1.67, -1.57, 0.0])
    
    # 4. 릴리즈
    node.do_release()
    
    print("="*40)
    print("✅ 완료! 로봇과 그리퍼를 동시에 제어했습니다!")
    print("="*40)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
