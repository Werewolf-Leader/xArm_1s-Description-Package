# XArm Remote - Quick Start Guide

## Prerequisites

Make sure you have built all packages:

```bash
cd ~/xarm_ws
colcon build
source install/setup.bash
```

## Quick Test

### Terminal 1: Launch the full system (MoveIt + Task Server)

```bash
source ~/xarm_ws/install/setup.bash
ros2 launch xarm_remote full_system.launch.py
```

This will start:
- RViz with MoveIt motion planning interface
- MoveIt move_group node
- ros2_control with mock hardware
- XArm task server

Wait until you see: **"You can start planning now!"**

### Terminal 2: Run the demo client

```bash
source ~/xarm_ws/install/setup.bash
ros2 run xarm_remote task_client.py
```

This will execute a demo sequence:
1. Move to home position
2. Open gripper
3. Move to pose_1
4. Close gripper

## Manual Testing

### Test Individual Commands

```bash
# Move to home
ros2 action send_goal /execute_task xarm_remote/action/ExecuteTask "{task_name: 'move_to_named', parameters: ['home']}" --feedback

# Move to pose_1
ros2 action send_goal /execute_task xarm_remote/action/ExecuteTask "{task_name: 'move_to_named', parameters: ['pose_1']}" --feedback

# Open gripper
ros2 action send_goal /execute_task xarm_remote/action/ExecuteTask "{task_name: 'control_gripper', gripper_position: -0.8}" --feedback

# Close gripper
ros2 action send_goal /execute_task xarm_remote/action/ExecuteTask "{task_name: 'control_gripper', gripper_position: 0.0}" --feedback
```

### Check Available Actions

```bash
# List action servers
ros2 action list

# Get action info
ros2 action info /execute_task
ros2 action info /pick_place
```

## Custom Python Script

Create your own script:

```python
#!/usr/bin/env python3
import rclpy
from xarm_remote.scripts.task_client import XArmTaskClient

def main():
    rclpy.init()
    client = XArmTaskClient()
    
    # Your custom sequence
    client.move_to_named_pose('home')
    client.control_gripper(-0.8)  # Open
    client.move_to_named_pose('pose_2')
    client.control_gripper(0.0)   # Close
    client.move_to_named_pose('home')
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Troubleshooting

### Task server not responding
- Make sure MoveIt is fully loaded (check for "You can start planning now!")
- Verify action servers are running: `ros2 action list`

### Planning failures
- Check joint limits in RViz
- Verify target poses are reachable
- Check for collisions in planning scene

### Gripper not moving
- Verify gripper controller is active: `ros2 control list_controllers`
- Check gripper limits: -0.87 to 0.0 for finger1

## Next Steps

1. Modify `task_client.py` for your specific tasks
2. Add custom named poses in xarm_moveit SRDF
3. Implement custom task types in task_server.cpp
4. Integrate with perception for object detection
5. Add collision objects to planning scene

## Available Named Poses

From xarm_moveit configuration:
- `home` - All joints at 0
- `pose_1` - Predefined pose 1
- `pose_2` - Predefined pose 2
- `pose_3` - Predefined pose 3

Gripper poses:
- `open` - Gripper fully open
- `close` - Gripper fully closed
- `home` - Gripper home position
