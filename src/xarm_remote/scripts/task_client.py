#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from xarm_remote.action import ExecuteTask, PickPlace
from geometry_msgs.msg import Pose, Point, Quaternion


class XArmTaskClient(Node):
    def __init__(self):
        super().__init__('xarm_task_client')
        
        self._execute_task_client = ActionClient(self, ExecuteTask, 'execute_task')
        self._pick_place_client = ActionClient(self, PickPlace, 'pick_place')
        
        self.get_logger().info('XArm Task Client initialized')

    def move_to_named_pose(self, pose_name):
        """Move arm to a named pose (e.g., 'home', 'pose_1')"""
        goal_msg = ExecuteTask.Goal()
        goal_msg.task_name = 'move_to_named'
        goal_msg.parameters = [pose_name]
        
        self.get_logger().info(f'Moving to named pose: {pose_name}')
        return self._send_execute_task_goal(goal_msg)

    def move_to_pose(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        """Move arm to a specific pose"""
        goal_msg = ExecuteTask.Goal()
        goal_msg.task_name = 'move_to_pose'
        
        goal_msg.target_pose.position.x = x
        goal_msg.target_pose.position.y = y
        goal_msg.target_pose.position.z = z
        goal_msg.target_pose.orientation.x = qx
        goal_msg.target_pose.orientation.y = qy
        goal_msg.target_pose.orientation.z = qz
        goal_msg.target_pose.orientation.w = qw
        
        self.get_logger().info(f'Moving to pose: ({x}, {y}, {z})')
        return self._send_execute_task_goal(goal_msg)

    def control_gripper(self, position):
        """Control gripper position (-0.87 to 0.0 for finger1)"""
        goal_msg = ExecuteTask.Goal()
        goal_msg.task_name = 'control_gripper'
        goal_msg.gripper_position = position
        
        self.get_logger().info(f'Setting gripper position: {position}')
        return self._send_execute_task_goal(goal_msg)

    def pick_and_place(self, pick_x, pick_y, pick_z, place_x, place_y, place_z,
                       approach_dist=0.1, retreat_dist=0.1):
        """Execute pick and place operation"""
        goal_msg = PickPlace.Goal()
        
        # Pick pose
        goal_msg.pick_pose.position.x = pick_x
        goal_msg.pick_pose.position.y = pick_y
        goal_msg.pick_pose.position.z = pick_z
        goal_msg.pick_pose.orientation.w = 1.0
        
        # Place pose
        goal_msg.place_pose.position.x = place_x
        goal_msg.place_pose.position.y = place_y
        goal_msg.place_pose.position.z = place_z
        goal_msg.place_pose.orientation.w = 1.0
        
        goal_msg.approach_distance = approach_dist
        goal_msg.retreat_distance = retreat_dist
        
        self.get_logger().info(f'Pick and place: ({pick_x}, {pick_y}, {pick_z}) -> ({place_x}, {place_y}, {place_z})')
        return self._send_pick_place_goal(goal_msg)

    def _send_execute_task_goal(self, goal_msg):
        """Send ExecuteTask goal and wait for result"""
        self._execute_task_client.wait_for_server()
        
        send_goal_future = self._execute_task_client.send_goal_async(
            goal_msg,
            feedback_callback=self._execute_task_feedback_callback
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
        self.get_logger().info(f'Result: {result.message} (time: {result.execution_time:.2f}s)')
        return result

    def _send_pick_place_goal(self, goal_msg):
        """Send PickPlace goal and wait for result"""
        self._pick_place_client.wait_for_server()
        
        send_goal_future = self._pick_place_client.send_goal_async(
            goal_msg,
            feedback_callback=self._pick_place_feedback_callback
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
        self.get_logger().info(f'Result: {result.message} (time: {result.total_time:.2f}s)')
        return result

    def _execute_task_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.current_state} ({feedback.progress*100:.0f}%)')

    def _pick_place_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Phase: {feedback.current_phase} ({feedback.progress*100:.0f}%)')


def main(args=None):
    rclpy.init(args=args)
    
    client = XArmTaskClient()
    
    try:
        # Example usage
        print("\n=== XArm Task Client Demo ===\n")
        
        # Move to home position
        print("1. Moving to home position...")
        client.move_to_named_pose('home')
        
        # Open gripper
        print("\n2. Opening gripper...")
        client.control_gripper(-0.8)
        
        # Move to a different named pose
        print("\n3. Moving to pose_1...")
        client.move_to_named_pose('pose_1')
        
        # Close gripper
        print("\n4. Closing gripper...")
        client.control_gripper(0.0)
        
        print("\n=== Demo completed ===\n")
        
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
