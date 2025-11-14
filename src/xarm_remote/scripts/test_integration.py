#!/usr/bin/env python3
"""
Integration test for xarm_remote task server.
This script tests all major functionality of the task server.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from xarm_remote.action import ExecuteTask, PickPlace
from geometry_msgs.msg import Pose
import sys


class IntegrationTest(Node):
    def __init__(self):
        super().__init__('integration_test')
        
        self._execute_task_client = ActionClient(self, ExecuteTask, 'execute_task')
        self._pick_place_client = ActionClient(self, PickPlace, 'pick_place')
        
        self.test_results = []
        
    def run_tests(self):
        """Run all integration tests"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('Starting xarm_remote Integration Tests')
        self.get_logger().info('=' * 60)
        
        tests = [
            ('Test 1: Move to Home', self.test_move_to_home),
            ('Test 2: Move to Named Pose', self.test_move_to_named),
            ('Test 3: Open Gripper', self.test_open_gripper),
            ('Test 4: Close Gripper', self.test_close_gripper),
            ('Test 5: Move to Pose_1', self.test_move_to_pose1),
            ('Test 6: Return to Home', self.test_return_home),
        ]
        
        for test_name, test_func in tests:
            self.get_logger().info(f'\n{test_name}...')
            try:
                result = test_func()
                if result:
                    self.get_logger().info(f'✅ {test_name} PASSED')
                    self.test_results.append((test_name, True))
                else:
                    self.get_logger().error(f'❌ {test_name} FAILED')
                    self.test_results.append((test_name, False))
            except Exception as e:
                self.get_logger().error(f'❌ {test_name} FAILED with exception: {e}')
                self.test_results.append((test_name, False))
        
        self.print_summary()
        
    def test_move_to_home(self):
        """Test moving to home position"""
        return self._send_named_pose_goal('home')
    
    def test_move_to_named(self):
        """Test moving to a named pose"""
        return self._send_named_pose_goal('pose_1')
    
    def test_open_gripper(self):
        """Test opening gripper"""
        return self._send_gripper_goal(-0.8)
    
    def test_close_gripper(self):
        """Test closing gripper"""
        return self._send_gripper_goal(0.0)
    
    def test_move_to_pose1(self):
        """Test moving to pose_1"""
        return self._send_named_pose_goal('pose_1')
    
    def test_return_home(self):
        """Test returning to home"""
        return self._send_named_pose_goal('home')
    
    def _send_named_pose_goal(self, pose_name):
        """Helper to send named pose goal"""
        if not self._execute_task_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available')
            return False
        
        goal_msg = ExecuteTask.Goal()
        goal_msg.task_name = 'move_to_named'
        goal_msg.parameters = [pose_name]
        
        send_goal_future = self._execute_task_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        
        result = result_future.result().result
        return result.success
    
    def _send_gripper_goal(self, position):
        """Helper to send gripper control goal"""
        if not self._execute_task_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available')
            return False
        
        goal_msg = ExecuteTask.Goal()
        goal_msg.task_name = 'control_gripper'
        goal_msg.gripper_position = position
        
        send_goal_future = self._execute_task_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        
        result = result_future.result().result
        return result.success
    
    def print_summary(self):
        """Print test summary"""
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('Test Summary')
        self.get_logger().info('=' * 60)
        
        passed = sum(1 for _, result in self.test_results if result)
        total = len(self.test_results)
        
        for test_name, result in self.test_results:
            status = '✅ PASS' if result else '❌ FAIL'
            self.get_logger().info(f'{status}: {test_name}')
        
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Results: {passed}/{total} tests passed')
        self.get_logger().info('=' * 60)
        
        return passed == total


def main(args=None):
    rclpy.init(args=args)
    
    test_node = IntegrationTest()
    
    try:
        all_passed = test_node.run_tests()
        exit_code = 0 if all_passed else 1
    except KeyboardInterrupt:
        test_node.get_logger().info('Test interrupted by user')
        exit_code = 1
    finally:
        test_node.destroy_node()
        rclpy.shutdown()
    
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
