# XArm Task Server - Number-Based Control

## Overview

This is a simplified task server that uses task numbers to control the xarm robot. Perfect for quick testing and simple automation.

## Task Numbers

### Arm Poses (0-3)
- **0** = Home position (all joints at 0)
- **1** = Pose 1
- **2** = Pose 2
- **3** = Pose 3

### Gripper Control (10-11)
- **10** = Open gripper
- **11** = Close gripper

## Quick Start

### Terminal 1: Launch the system

```bash
source ~/xarm_ws/install/setup.bash
ros2 launch xarm_remote full_system_number.launch.py
```

Wait for: **"Task Server ready! Send task numbers:"**

### Terminal 2: Send commands

```bash
source ~/xarm_ws/install/setup.bash

# Move to home
ros2 action send_goal /task_server_number xarm_remote/action/XarmTask "{task_number: 0}" --feedback

# Open gripper
ros2 action send_goal /task_server_number xarm_remote/action/XarmTask "{task_number: 10}" --feedback

# Move to pose 1
ros2 action send_goal /task_server_number xarm_remote/action/XarmTask "{task_number: 1}" --feedback

# Close gripper
ros2 action send_goal /task_server_number xarm_remote/action/XarmTask "{task_number: 11}" --feedback

# Move to pose 2
ros2 action send_goal /task_server_number xarm_remote/action/XarmTask "{task_number: 2}" --feedback

# Move to pose 3
ros2 action send_goal /task_server_number xarm_remote/action/XarmTask "{task_number: 3}" --feedback
```

## Using Python Client

### Run the demo:
```bash
ros2 run xarm_remote task_client_number.py
```

### Custom Python script:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from xarm_remote.action import XarmTask

rclpy.init()
node = Node('my_controller')
client = ActionClient(node, XarmTask, 'task_server_number')

# Send task
goal = XarmTask.Goal()
goal.task_number = 0  # Home
client.wait_for_server()
future = client.send_goal_async(goal)
rclpy.spin_until_future_complete(node, future)

# Get result
goal_handle = future.result()
result_future = goal_handle.get_result_async()
rclpy.spin_until_future_complete(node, result_future)
result = result_future.result().result

print(f"Success: {result.success}, Message: {result.message}")

rclpy.shutdown()
```

## Example Sequences

### Pick and Place Simulation
```bash
# Go to home
ros2 action send_goal /task_server_number xarm_remote/action/XarmTask "{task_number: 0}"

# Open gripper
ros2 action send_goal /task_server_number xarm_remote/action/XarmTask "{task_number: 10}"

# Move to pick position (pose 1)
ros2 action send_goal /task_server_number xarm_remote/action/XarmTask "{task_number: 1}"

# Close gripper (grasp)
ros2 action send_goal /task_server_number xarm_remote/action/XarmTask "{task_number: 11}"

# Move to place position (pose 2)
ros2 action send_goal /task_server_number xarm_remote/action/XarmTask "{task_number: 2}"

# Open gripper (release)
ros2 action send_goal /task_server_number xarm_remote/action/XarmTask "{task_number: 10}"

# Return home
ros2 action send_goal /task_server_number xarm_remote/action/XarmTask "{task_number: 0}"
```

### Testing All Poses
```bash
for i in 0 1 2 3 0; do
  echo "Moving to task $i"
  ros2 action send_goal /task_server_number xarm_remote/action/XarmTask "{task_number: $i}"
  sleep 2
done
```

## Action Interface

```
# Goal
int32 task_number

---
# Result
bool success
string message

---
# Feedback
string status
```

## Checking Action Status

```bash
# List actions
ros2 action list

# Get action info
ros2 action info /task_server_number

# Show action interface
ros2 interface show xarm_remote/action/XarmTask
```

## Troubleshooting

### Server not responding
```bash
# Check if server is running
ros2 node list | grep task_server_number

# Check action server
ros2 action list
```

### Invalid task number
Valid ranges:
- 0-3: Arm poses
- 10-11: Gripper control

### Planning failures
- Verify MoveIt is fully loaded
- Check RViz for collision warnings
- Ensure target poses are reachable

## Integration with Other Systems

### From C++
```cpp
#include "rclcpp_action/rclcpp_action.hpp"
#include "xarm_remote/action/xarm_task.hpp"

auto client = rclcpp_action::create_client<xarm_remote::action::XarmTask>(
    node, "task_server_number");

auto goal = xarm_remote::action::XarmTask::Goal();
goal.task_number = 0;
client->async_send_goal(goal);
```

### From ROS2 Service/Topic
You can wrap this in a service or subscribe to topics and trigger tasks based on messages.

## Advantages of Number-Based Control

1. **Simple**: Just send a number
2. **Fast**: No need to construct complex messages
3. **Easy to remember**: 0=home, 1-3=poses, 10-11=gripper
4. **Scriptable**: Easy to use in bash scripts
5. **Debuggable**: Quick to test individual tasks

## Next Steps

1. Add more task numbers for custom poses
2. Create task sequences (e.g., task 100 = pick and place)
3. Add parameters for dynamic poses
4. Integrate with sensors for automated workflows
