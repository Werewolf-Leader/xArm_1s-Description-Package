#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from xarm_remote.action import XarmTask


class XArmTaskClientNumber(Node):
    def __init__(self):
        super().__init__('xarm_task_client_number')
        
        self._action_client = ActionClient(self, XarmTask, 'task_server_number')
        
        self.get_logger().info('XArm Task Client (Number-based) initialized')

    def send_task(self, task_number):
        """Send task number to the server"""
        goal_msg = XarmTask.Goal()
        goal_msg.task_number = task_number
        
        task_names = {
            0: "Home",
            1: "Pose 1",
            2: "Pose 2",
            3: "Pose 3",
            10: "Open Gripper",
            11: "Close Gripper"
        }
        
        task_name = task_names.get(task_number, f"Task {task_number}")
        self.get_logger().info(f'Sending task: {task_name} (number: {task_number})')
        
        self._action_client.wait_for_server()
        
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return None
        
        self.get_logger().info('Goal accepted')
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        
        if result.success:
            self.get_logger().info(f'✅ SUCCESS: {result.message}')
        else:
            self.get_logger().error(f'❌ FAILED: {result.message}')
        
        return result

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.status}')


def main(args=None):
    rclpy.init(args=args)
    
    client = XArmTaskClientNumber()
    
    try:
        print("\n=== XArm Task Client Demo (Number-based) ===\n")
        
        # Demo sequence
        print("Task 0: Moving to home position...")
        client.send_task(0)
        
        print("\nTask 10: Opening gripper...")
        client.send_task(10)
        
        print("\nTask 1: Moving to pose_1...")
        client.send_task(1)
        
        print("\nTask 11: Closing gripper...")
        client.send_task(11)
        
        print("\nTask 2: Moving to pose_2...")
        client.send_task(2)
        
        print("\nTask 0: Returning to home...")
        client.send_task(0)
        
        print("\n=== Demo completed ===\n")
        
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
