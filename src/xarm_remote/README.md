# xarm_remote

Remote task server for xarm robot manipulation using MoveIt2.

## Overview

This package provides a high-level task server for controlling the xarm robot. It exposes action interfaces for common manipulation tasks like moving to poses, controlling the gripper, and executing pick-and-place operations.

## Features

- **ExecuteTask Action**: Execute various manipulation tasks
  - Move to named poses (home, pose_1, pose_2, etc.)
  - Move to specific Cartesian poses
  - Control gripper position
  
- **PickPlace Action**: Automated pick and place operations
  - Approach, grasp, lift, move, place, and retreat phases
  - Configurable approach and retreat distances
  - Real-time feedback on operation progress

## Installation

```bash
cd ~/xarm_ws
colcon build --packages-select xarm_remote
source install/setup.bash
```

## Usage

### Launch Task Server Only

```bash
ros2 launch xarm_remote task_server.launch.py
```

### Launch Full System (MoveIt + Task Server)

```bash
ros2 launch xarm_remote full_system.launch.py
```

### Using the Python Client

```bash
# Run the demo client
ros2 run xarm_remote task_client.py
```

### Python API Examples

```python
from xarm_remote.scripts.task_client import XArmTaskClient
import rclpy

rclpy.init()
client = XArmTaskClient()

# Move to home position
client.move_to_named_pose('home')

# Move to specific pose
client.move_to_pose(x=0.2, y=0.1, z=0.3)

# Control gripper (open)
client.control_gripper(-0.8)

# Control gripper (close)
client.control_gripper(0.0)

# Pick and place
client.pick_and_place(
    pick_x=0.2, pick_y=0.1, pick_z=0.1,
    place_x=0.2, place_y=-0.1, place_z=0.1,
    approach_dist=0.1,
    retreat_dist=0.1
)
```

### Using Action Client from Command Line

```bash
# Send ExecuteTask goal
ros2 action send_goal /execute_task xarm_remote/action/ExecuteTask "{task_name: 'move_to_named', parameters: ['home']}"

# Send PickPlace goal
ros2 action send_goal /pick_place xarm_remote/action/PickPlace "{pick_pose: {position: {x: 0.2, y: 0.1, z: 0.1}, orientation: {w: 1.0}}, place_pose: {position: {x: 0.2, y: -0.1, z: 0.1}, orientation: {w: 1.0}}, approach_distance: 0.1, retreat_distance: 0.1}"
```

## Action Interfaces

### ExecuteTask

**Goal:**
- `task_name` (string): Task to execute ("move_to_pose", "move_to_named", "control_gripper")
- `parameters` (string[]): Task-specific parameters
- `target_pose` (Pose): Target pose for move_to_pose task
- `gripper_position` (float32): Gripper position for control_gripper task

**Result:**
- `success` (bool): Whether task succeeded
- `message` (string): Result message
- `execution_time` (float32): Time taken to execute

**Feedback:**
- `current_state` (string): Current execution state
- `progress` (float32): Progress from 0.0 to 1.0

### PickPlace

**Goal:**
- `pick_pose` (Pose): Pose to pick object from
- `place_pose` (Pose): Pose to place object at
- `approach_distance` (float32): Distance to approach before grasping
- `retreat_distance` (float32): Distance to retreat after grasping/placing

**Result:**
- `success` (bool): Whether operation succeeded
- `message` (string): Result message
- `total_time` (float32): Total time taken

**Feedback:**
- `current_phase` (string): Current phase of operation
- `progress` (float32): Progress from 0.0 to 1.0

## Architecture

```
┌─────────────────┐
│  Task Client    │
│  (Python/C++)   │
└────────┬────────┘
         │ Action Interface
         ▼
┌─────────────────┐
│  Task Server    │
│  (C++)          │
└────────┬────────┘
         │ MoveIt API
         ▼
┌─────────────────┐
│  MoveIt         │
│  move_group     │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  ros2_control   │
│  (Mock/Real HW) │
└─────────────────┘
```

## Dependencies

- rclcpp
- rclcpp_action
- moveit_ros_planning_interface
- geometry_msgs
- xarm_moveit

## License

BSD

## Author

Mukul Sharma (imsharma@protonmail.com)
