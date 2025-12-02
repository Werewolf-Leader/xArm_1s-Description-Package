# XArm 1s ROS2 Complete Package

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-green)](https://docs.ros.org/en/jazzy/)
[![License: CC BY-NC 4.0](https://img.shields.io/badge/License-CC%20BY--NC%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc/4.0/)

A professional, production-ready ROS2 package suite for the XArm 1s robotic manipulator. This workspace provides complete URDF/XACRO descriptions, MoveIt2 motion planning, ros2_control integration, and a high-level task execution system for both simulation and hardware control.

![XArm Robot](https://img.shields.io/badge/DOF-5+2-orange) ![Build](https://img.shields.io/badge/build-passing-brightgreen) ![MoveIt2](https://img.shields.io/badge/MoveIt2-Enabled-purple)


<video controls src="eYRC-KC#5167_Task1A (online-video-cutter.com)(2).mp4" title="Title"></video>

## 🌟 Features

- ✅ **Complete Robot Description**: 5-DOF arm + 2-DOF gripper with accurate kinematics
- ✅ **MoveIt2 Integration**: Full motion planning with collision detection and path planning
- ✅ **Task Execution System**: High-level action server for pick-and-place operations
- ✅ **Alexa Voice Control**: Voice-controlled robot operations via Amazon Alexa integration
- ✅ **ROS2 Control Integration**: Full ros2_control support for simulation and hardware
- ✅ **Gazebo Simulation**: Ready-to-use Gazebo Classic integration with physics
- ✅ **RViz2 Visualization**: Interactive visualization with joint state control
- ✅ **Modern Architecture**: Python-based launch files, event-driven controller spawning
- ✅ **Secure Configuration**: Environment-based secrets management for production deployment
- ✅ **Well Documented**: Comprehensive guides, quickstart tutorials, and examples
- ✅ **Production Ready**: Tested on ROS2 Humble and Jazzy

## 📋 Table of Contents

- [Packages Overview](#packages-overview)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Usage](#usage)
- [Robot Specifications](#robot-specifications)
- [Documentation](#documentation)
- [Contributing](#contributing)
- [License](#license)

## 📦 Packages Overview

This workspace contains three main packages:

### 1. **xarm_description**
Core robot description package with URDF/XACRO files, ros2_control configuration, and basic visualization/simulation capabilities.

### 2. **xarm_moveit**
MoveIt2 configuration package providing motion planning, collision detection, and trajectory execution for the XArm 1s.

### 3. **xarm_remote**
High-level task execution system with action servers for complex operations like pick-and-place. Includes both string-based and number-based task interfaces, plus Alexa voice control integration for hands-free robot operation.

## 🚀 Installation

### Prerequisites

- ROS2 Humble or Jazzy
- Ubuntu 22.04 (for Humble) or Ubuntu 24.04 (for Jazzy)
- Gazebo Classic (for simulation)

### Install Dependencies

```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-gazebo-ros2-control \
                 ros-${ROS_DISTRO}-joint-state-broadcaster \
                 ros-${ROS_DISTRO}-joint-trajectory-controller \
                 ros-${ROS_DISTRO}-robot-state-publisher \
                 ros-${ROS_DISTRO}-joint-state-publisher-gui \
                 ros-${ROS_DISTRO}-xacro \
                 ros-${ROS_DISTRO}-gazebo-ros-pkgs \
                 ros-${ROS_DISTRO}-moveit \
                 ros-${ROS_DISTRO}-moveit-ros-planning-interface \
                 ros-${ROS_DISTRO}-moveit-visual-tools
```

### Build from Source

```bash
# Create workspace
mkdir -p ~/xarm_ws/src
cd ~/xarm_ws/src

# Clone repository
git clone -b moveit-configured https://github.com/Werewolf-Leader/xArm_1s-Description-Package.git .

# Build
cd ~/xarm_ws
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

## ⚡ Quick Start

### 1. Basic Visualization (RViz2)

```bash
source ~/xarm_ws/install/setup.bash
ros2 launch xarm_description display.launch.py
```

Use the Joint State Publisher GUI to interactively control the robot joints!

### 2. MoveIt2 Demo (Motion Planning)

```bash
source ~/xarm_ws/install/setup.bash
ros2 launch xarm_moveit demo.launch.py
```

Use the MoveIt2 RViz interface to plan and execute trajectories with collision detection!

### 3. Full System with Task Server

```bash
source ~/xarm_ws/install/setup.bash
ros2 launch xarm_remote full_system.launch.py
```

Then send high-level commands:

```bash
# Pick and place operation
ros2 action send_goal /xarm_task xarm_remote/action/XarmTask \
  "{task_name: 'pick_place', target_position: {x: 0.2, y: 0.1, z: 0.15}}"

# Or use the Python client
ros2 run xarm_remote task_client.py
```

### 4. Alexa Voice Control

Control your robot with voice commands! First, configure your Alexa Skill ID:

```bash
# Copy environment template
cd ~/xarm_ws/src/xarm_remote
cp .env.example .env

# Edit .env and add your Alexa Skill ID
nano .env
```

Then launch the Alexa interface:

```bash
source ~/xarm_ws/install/setup.bash
export ALEXA_SKILL_ID=your-skill-id-here  # or set in .env
ros2 launch xarm_remote remote_interface.launch.py
```

See [Alexa Setup Guide](src/xarm_remote/README_ALEXA_SETUP.md) for detailed configuration.

**Voice Commands:**
- "Alexa, ask robot to wake up" - Initialize robot
- "Alexa, ask robot to pick" - Execute pick and place
- "Alexa, ask robot to sleep" - Return to home position

### 5. Gazebo Simulation

```bash
source ~/xarm_ws/install/setup.bash
ros2 launch xarm_description gazebo.launch.py
```

## 📖 Usage

### xarm_description Package

Basic robot description and visualization:
- `display.launch.py` - RViz2 visualization with joint control
- `gazebo.launch.py` - Gazebo simulation with ros2_control
- `controller.launch.py` - Standalone controller manager

See [xarm_description README](src/xarm_description/README.md) for details.

### xarm_moveit Package

MoveIt2 motion planning:
- `demo.launch.py` - Complete MoveIt2 demo with RViz
- `move_group.launch.py` - MoveIt2 move_group node
- `moveit_rviz.launch.py` - RViz with MoveIt2 plugin

### xarm_remote Package

High-level task execution and voice control:
- `full_system.launch.py` - Complete system (MoveIt2 + Task Server)
- `full_system_number.launch.py` - Number-based task interface
- `remote_interface.launch.py` - Alexa voice control interface
- `task_server.launch.py` - Standalone task server
- `task_client.py` - Python client for sending tasks

See [xarm_remote README](src/xarm_remote/README.md) and [Alexa Setup Guide](src/xarm_remote/README_ALEXA_SETUP.md) for detailed tutorials.

## 🤖 Robot Specifications

- **Degrees of Freedom**: 7 (5 arm joints + 2 gripper joints)
- **Controller Type**: Joint Trajectory Controller
- **Control Interfaces**: Position, Velocity, Effort
- **Workspace**: ~300mm reach
- **Payload**: Lightweight objects

### Joint Configuration

| Joint | Type | Range |
|-------|------|-------|
| base_to_link1 | Revolute | ±180° |
| link1_to_link2 | Revolute | ±90° |
| link2_to_link3 | Revolute | ±130° |
| linkt_to_link5 | Revolute | ±130° |
| link5_to_gripperbase | Revolute | ±180° |
| gripper_finger1_joint | Revolute | -50° to 0° |
| gripper_finger2_joint | Revolute | 0° to 50° |

## 📚 Documentation

### Package Documentation
- **[xarm_description README](src/xarm_description/README.md)** - Robot description and basic usage
- **[xarm_remote README](src/xarm_remote/README.md)** - Complete task system documentation
- **[Alexa Setup Guide](src/xarm_remote/README_ALEXA_SETUP.md)** - Voice control configuration

### Project Documentation
- **[License](src/xarm_description/LICENSE)** - CC BY-NC 4.0

## 🛠️ Troubleshooting

### Package Not Found

Make sure to source the workspace:
```bash
source ~/xarm_ws/install/setup.bash
```

Add to `~/.bashrc` for automatic sourcing:
```bash
echo "source ~/xarm_ws/install/setup.bash" >> ~/.bashrc
```

### Alexa Interface Not Starting

Make sure you've set the `ALEXA_SKILL_ID` environment variable:
```bash
export ALEXA_SKILL_ID=your-skill-id-here
```

Or create a `.env` file in `src/xarm_remote/` with your Skill ID. See [Alexa Setup Guide](src/xarm_remote/README_ALEXA_SETUP.md).

### More Issues?

Check the [Troubleshooting Section](src/xarm_description/README.md#troubleshooting) in the package README.

## 🤝 Contributing

Contributions are welcome! Please open an issue or pull request for:

- Hardware interface implementation
- Additional task types and behaviors
- Performance optimizations
- Additional examples and tutorials
- Bug fixes and improvements
- Documentation enhancements

## 📜 License

This project is licensed under the Creative Commons Attribution-NonCommercial 4.0 International License (CC BY-NC 4.0).

- ✅ Free for educational and research use
- ✅ Can be modified and shared with attribution
- ❌ Cannot be used for commercial purposes

See [LICENSE](src/xarm_description/LICENSE) for full details.

## 🙏 Acknowledgments

- Original URDF model: Toshinori Kitamura (2018)
- ROS2 conversion and enhancement: 2025
- Built with ROS2 and ros2_control framework

## 📞 Contact & Support

- **Issues**: [GitHub Issues](https://github.com/Werewolf-Leader/xArm_1s-ROS2-URDF-package/issues)
- **Discussions**: [GitHub Discussions](https://github.com/Werewolf-Leader/xArm_1s-ROS2-URDF-package/discussions)

## ⭐ Star History

If you find this project useful, please give it a star! It helps others discover the project.

---

**Made with ❤️ for the ROS2 community**
