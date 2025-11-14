# XArm Task Server - Quick Reference Card

## 🚀 Quick Start

```bash
# Terminal 1: Launch
ros2 launch xarm_remote full_system_number.launch.py

# Terminal 2: Send commands
ros2 action send_goal /task_server_number xarm_remote/action/XarmTask "{task_number: 0}"
```

## 📋 Task Numbers

| Number | Action | Description |
|--------|--------|-------------|
| **0** | Home | All joints to zero position |
| **1** | Pose 1 | Predefined pose 1 |
| **2** | Pose 2 | Predefined pose 2 |
| **3** | Pose 3 | Predefined pose 3 |
| **10** | Open | Open gripper |
| **11** | Close | Close gripper |

## 💻 Command Examples

### Move to Home
```bash
ros2 action send_goal /task_server_number xarm_remote/action/XarmTask "{task_number: 0}" --feedback
```

### Open Gripper
```bash
ros2 action send_goal /task_server_number xarm_remote/action/XarmTask "{task_number: 10}" --feedback
```

### Move to Pose 1
```bash
ros2 action send_goal /task_server_number xarm_remote/action/XarmTask "{task_number: 1}" --feedback
```

### Close Gripper
```bash
ros2 action send_goal /task_server_number xarm_remote/action/XarmTask "{task_number: 11}" --feedback
```

## 🐍 Python Usage

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from xarm_remote.action import XarmTask

rclpy.init()
node = Node('my_node')
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

## 🔄 Common Sequences

### Pick and Place
```bash
ros2 action send_goal /task_server_number xarm_remote/action/XarmTask "{task_number: 0}"  # Home
ros2 action send_goal /task_server_number xarm_remote/action/XarmTask "{task_number: 10}" # Open
ros2 action send_goal /task_server_number xarm_remote/action/XarmTask "{task_number: 1}"  # Pick
ros2 action send_goal /task_server_number xarm_remote/action/XarmTask "{task_number: 11}" # Close
ros2 action send_goal /task_server_number xarm_remote/action/XarmTask "{task_number: 2}"  # Place
ros2 action send_goal /task_server_number xarm_remote/action/XarmTask "{task_number: 10}" # Open
ros2 action send_goal /task_server_number xarm_remote/action/XarmTask "{task_number: 0}"  # Home
```

### Test All Poses
```bash
for i in 0 1 2 3 0; do
  ros2 action send_goal /task_server_number xarm_remote/action/XarmTask "{task_number: $i}"
  sleep 2
done
```

## 🔍 Debugging

### Check if server is running
```bash
ros2 node list | grep task_server_number
```

### List available actions
```bash
ros2 action list
```

### Show action interface
```bash
ros2 interface show xarm_remote/action/XarmTask
```

### Check action info
```bash
ros2 action info /task_server_number
```

## 📦 Available Executables

```bash
# Number-based task server
ros2 run xarm_remote task_server_number

# Number-based Python client (demo)
ros2 run xarm_remote task_client_number.py

# Original task server
ros2 run xarm_remote task_server

# Original Python client
ros2 run xarm_remote task_client.py
```

## 🎯 Launch Files

```bash
# Full system with number-based server
ros2 launch xarm_remote full_system_number.launch.py

# Number-based server only
ros2 launch xarm_remote task_server_number.launch.py

# Full system with original server
ros2 launch xarm_remote full_system.launch.py

# Original server only
ros2 launch xarm_remote task_server.launch.py
```

## ⚠️ Troubleshooting

| Problem | Solution |
|---------|----------|
| Server not responding | Check if MoveIt is fully loaded |
| Invalid task number | Use 0-3 for arm, 10-11 for gripper |
| Planning failed | Check RViz for collisions |
| Goal rejected | Verify server is running |

## 📚 Documentation

- Full guide: `TASK_NUMBER_GUIDE.md`
- Summary: `TASK_SERVER_NUMBER_SUMMARY.md`
- Package README: `README.md`
- Quick start: `QUICKSTART.md`

---

**Remember**: Task numbers are simple and fast!
- 0-3 = Arm poses
- 10-11 = Gripper control
