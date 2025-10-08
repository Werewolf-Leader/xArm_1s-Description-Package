#!/bin/bash
# Quick reference commands for XArm ROS2 package
# Source this file or copy commands as needed

# Build the workspace
alias xarm-build='cd ~/xarm1s_ws && colcon build --symlink-install'

# Clean build
alias xarm-clean='cd ~/xarm1s_ws && rm -rf build/ install/ log/ && colcon build --symlink-install'

# Source workspace
alias xarm-source='source ~/xarm1s_ws/install/setup.bash'

# Launch RViz visualization
alias xarm-viz='ros2 launch xarm_description display.launch.py'

# Launch Gazebo simulation
alias xarm-sim='ros2 launch xarm_description gazebo.launch.py'

# List controllers
alias xarm-controllers='ros2 control list_controllers'

# List topics
alias xarm-topics='ros2 topic list'

# Echo joint states
alias xarm-joints='ros2 topic echo /joint_states'

# Check URDF
alias xarm-check='check_urdf ~/xarm1s_ws/install/xarm_description/share/xarm_description/urdf/xarm.xacro'

# View TF tree
alias xarm-tf='ros2 run tf2_tools view_frames'

echo "XArm aliases loaded! Available commands:"
echo "  xarm-build        - Build the workspace"
echo "  xarm-clean        - Clean and rebuild"
echo "  xarm-source       - Source the workspace"
echo "  xarm-viz          - Launch RViz visualization"
echo "  xarm-sim          - Launch Gazebo simulation"
echo "  xarm-controllers  - List active controllers"
echo "  xarm-topics       - List all topics"
echo "  xarm-joints       - Monitor joint states"
echo "  xarm-check        - Validate URDF"
echo "  xarm-tf           - View TF tree"
