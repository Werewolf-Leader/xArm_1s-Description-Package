# XArm Robot Description Package

A ROS2-compatible robot description package for the XArm robotic manipulator with gripper.

## Package Information

- **Package Name**: xarm_description
- **ROS2 Compatibility**: Humble, Jazzy
- **Build System**: ament_cmake
- **License**: Creative Commons Attribution-NonCommercial 4.0 (CC BY-NC 4.0)

## Features

- Complete URDF/XACRO description of XArm robot with 5-DOF arm + 2-DOF gripper
- ROS2 Control integration for simulation and hardware control
- Gazebo Classic simulation support
- RViz2 visualization
- Joint trajectory controller for coordinated motion
- Launch files for visualization and simulation

## Dependencies

### Build Dependencies
- ament_cmake
- urdf
- xacro

### Runtime Dependencies
- robot_state_publisher
- joint_state_publisher
- joint_state_publisher_gui
- rviz2
- gazebo_ros
- gazebo_ros2_control
- controller_manager
- joint_state_broadcaster
- joint_trajectory_controller

## Installation

1. Create a ROS2 workspace (if you haven't already):
```bash
mkdir -p ~/xarm1s_ws/src
cd ~/xarm1s_ws/src
```

2. Clone this repository into the src folder

3. Install dependencies:
```bash
cd ~/xarm1s_ws
rosdep install --from-paths src --ignore-src -r -y
```

4. Build the workspace:
```bash
colcon build --symlink-install
```

5. Source the workspace:
```bash
source install/setup.bash
```

**Important**: You must source the workspace in every new terminal session before using the package. To make this automatic, add this line to your `~/.bashrc`:
```bash
echo "source ~/xarm1s_ws/install/setup.bash" >> ~/.bashrc
```

Alternatively, use the provided setup script:
```bash
source ~/xarm1s_ws/setup_xarm.sh
```

## Usage

**Note**: Before running any commands below, make sure you've sourced the workspace:
```bash
cd ~/xarm1s_ws
source install/setup.bash
```

### Visualize Robot in RViz2

To visualize the robot model in RViz2 with joint state publisher GUI:

```bash
ros2 launch xarm_description display.launch.py
```

This will launch:
- Robot state publisher
- Joint state publisher GUI (to manually control joints)
- RViz2 for visualization

### Simulate Robot in Gazebo

To launch the robot in Gazebo simulation with controllers:

```bash
ros2 launch xarm_description gazebo.launch.py
```

This will:
- Start Gazebo Classic
- Spawn the XArm robot
- Load ros2_control controllers
- Enable joint trajectory control

### Control the Robot

After launching Gazebo, you can send trajectory commands to the robot:

```bash
# List available controllers
ros2 control list_controllers

# Send test trajectory (example)
ros2 action send_goal /xarm_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [base_to_link1, link1_to_link2, link2_to_link3, \
  linkt_to_link5, link5_to_gripperbase, gripper_finger1_joint, gripper_finger2_joint], \
  points: [{positions: [0.5, 0.5, 0.5, 0.5, 0.5, -0.3, 0.3], time_from_start: {sec: 2}}]}}"
```

## Robot Specifications

### Joint Configuration

| Joint Name | Type | Range (rad) | Parent Link | Child Link |
|------------|------|-------------|-------------|------------|
| base_to_link1 | Revolute | -π to π | base_link | Link_1_1 |
| link1_to_link2 | Revolute | -π/2 to π/2 | Link_1_1 | Link_2_1 |
| link2_to_link3 | Revolute | -2.27 to 2.27 | Link_2_1 | Link_3_1 |
| linkt_to_link5 | Revolute | -2.27 to 2.27 | Link_4_1 | Link_5_1 |
| link5_to_gripperbase | Revolute | -π to π | Link_5_1 | gripper_base_1 |
| gripper_finger1_joint | Revolute | -0.87 to 0 | gripper_base_1 | finger_1_1 |
| gripper_finger2_joint | Revolute | 0 to 0.87 | gripper_base_1 | finger_2_1 |

### Controllers

The package includes:
- **joint_state_broadcaster**: Publishes joint states to /joint_states topic
- **xarm_trajectory_controller**: Joint trajectory controller for coordinated multi-joint motion

## File Structure

```
xarm_description/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   ├── controller.launch.py      # Standalone controller launcher
│   ├── controller.yaml            # Controller configuration
│   ├── display.launch.py          # RViz2 visualization
│   ├── gazebo.launch.py           # Gazebo simulation
│   └── urdf.rviz                  # RViz configuration
├── meshes/                        # STL mesh files
│   ├── base_link.stl
│   ├── Link_1_1.stl
│   ├── Link_2_1.stl
│   ├── Link_3_1.stl
│   ├── Link_4_1.stl
│   ├── Link_5_1.stl
│   ├── gripper_base_1.stl
│   ├── finger_1_1.stl
│   ├── finger_2_1.stl
│   ├── Gripper_end_1_1.stl
│   └── Gripper_end_2_1.stl
└── urdf/
    ├── xarm.xacro                 # Main robot description
    ├── xarm.gazebo                # Gazebo-specific properties
    ├── xarm.ros2_control.xacro    # ros2_control configuration
    └── materials.xacro            # Material definitions
```

## ROS2 Control Architecture

This package uses ros2_control with the following architecture:

- **Hardware Interface**: GazeboSystem (for simulation)
- **Command Interfaces**: position, velocity, effort
- **State Interfaces**: position, velocity, effort
- **Controller Type**: JointTrajectoryController

## Troubleshooting

### Package Not Found Error

If you see an error like `Package 'xarm_description' not found`:

**Solution**: You need to source the workspace in your current terminal session:
```bash
cd ~/xarm1s_ws
source install/setup.bash
```

This must be done in **every new terminal window** before launching. To avoid this, add the source command to your `~/.bashrc` as mentioned in the Installation section.

### Build Errors

If you encounter build errors:
```bash
# Clean the workspace
cd ~/xarm1s_ws
rm -rf build/ install/ log/

# Rebuild
colcon build --symlink-install
```

### Missing Dependencies

Install all ROS2 dependencies:
```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-gazebo-ros2-control \
                 ros-${ROS_DISTRO}-joint-state-broadcaster \
                 ros-${ROS_DISTRO}-joint-trajectory-controller \
                 ros-${ROS_DISTRO}-robot-state-publisher \
                 ros-${ROS_DISTRO}-joint-state-publisher-gui \
                 ros-${ROS_DISTRO}-xacro \
                 ros-${ROS_DISTRO}-gazebo-ros-pkgs
```

Or use rosdep to install dependencies automatically:
```bash
cd ~/xarm1s_ws
rosdep install --from-paths src --ignore-src -r -y
```

### Gazebo Doesn't Start

Make sure Gazebo is properly installed:
```bash
sudo apt install ros-${ROS_DISTRO}-gazebo-ros-pkgs
```

If Gazebo crashes or doesn't spawn the robot:
1. Check that gazebo_ros2_control is installed
2. Verify the URDF processes correctly: `xacro src/xarm_description/urdf/xarm.xacro`
3. Check controller configuration in `launch/controller.yaml`

### RViz Shows No Robot Model

If RViz2 opens but shows no robot:
1. Ensure you have the RobotModel display added (it should be by default)
2. Check that the topic `/robot_description` is being published: `ros2 topic list | grep robot_description`
3. Verify joint states are being published: `ros2 topic echo /joint_states`
4. Check RViz Fixed Frame is set to `base_link`

## Contributing

Contributions are welcome! Please ensure:
- Code follows ROS2 best practices
- Launch files are tested with both Humble and Jazzy
- URDF/XACRO files pass `check_urdf` validation

## Quick Start Guide

For first-time users, follow these steps:

1. **Build the workspace:**
   ```bash
   cd ~/xarm1s_ws
   colcon build --symlink-install
   ```

2. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

3. **Visualize the robot:**
   ```bash
   ros2 launch xarm_description display.launch.py
   ```

4. **Use the GUI sliders** in the Joint State Publisher window to move the robot joints and see the robot move in RViz2!


## Future Improvements

- [ ] Add MoveIt2 configuration
- [ ] Implement real hardware interface
- [ ] Add more sophisticated gripper control
- [ ] Create demo scripts for common motions
- [ ] Add unit tests

## References

- [ROS2 Control Documentation](https://control.ros.org/)
- [Gazebo ROS2 Control](https://github.com/ros-simulation/gazebo_ros2_control)
- [URDF Tutorial](http://wiki.ros.org/urdf/Tutorials)

## License

This project is licensed under the **Creative Commons Attribution-NonCommercial 4.0 International License (CC BY-NC 4.0)**.

### You are free to:
- **Share** — copy and redistribute the material in any medium or format
- **Adapt** — remix, transform, and build upon the material

### Under the following terms:
- **Attribution** — You must give appropriate credit, provide a link to the license, and indicate if changes were made
- **NonCommercial** — You may not use the material for commercial purposes

For more details, see the [LICENSE](LICENSE) file or visit: https://creativecommons.org/licenses/by-nc/4.0/
