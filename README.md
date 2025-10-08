# XArm 1s ROS2 URDF Package

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-green)](https://docs.ros.org/en/jazzy/)
[![License: CC BY-NC 4.0](https://img.shields.io/badge/License-CC%20BY--NC%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc/4.0/)

A professional, production-ready ROS2 package for the XArm 1s robotic manipulator. This package provides complete URDF/XACRO descriptions, ros2_control integration, and simulation capabilities for both Gazebo and RViz2.

![XArm Robot](https://img.shields.io/badge/DOF-5+2-orange) ![Build](https://img.shields.io/badge/build-passing-brightgreen)

## 🌟 Features

- ✅ **Complete Robot Description**: 5-DOF arm + 2-DOF gripper with accurate kinematics
- ✅ **ROS2 Control Integration**: Full ros2_control support for simulation and hardware
- ✅ **Gazebo Simulation**: Ready-to-use Gazebo Classic integration with physics
- ✅ **RViz2 Visualization**: Interactive visualization with joint state control
- ✅ **Modern Architecture**: Python-based launch files, event-driven controller spawning
- ✅ **Well Documented**: Comprehensive README, troubleshooting guides, and examples
- ✅ **Production Ready**: Tested on ROS2 Humble and Jazzy

## 📋 Table of Contents

- [Installation](#installation)
- [Quick Start](#quick-start)
- [Usage](#usage)
- [Robot Specifications](#robot-specifications)
- [Documentation](#documentation)
- [Contributing](#contributing)
- [License](#license)

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
                 ros-${ROS_DISTRO}-gazebo-ros-pkgs
```

### Build from Source

```bash
# Create workspace
mkdir -p ~/xarm_ws/src
cd ~/xarm_ws/src

# Clone repository
git clone https://github.com/Werewolf-Leader/xArm_1s-ROS2-URDF-package.git xarm_description

# Build
cd ~/xarm_ws
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

## ⚡ Quick Start

### Visualize in RViz2

```bash
source ~/xarm_ws/install/setup.bash
ros2 launch xarm_description display.launch.py
```

Use the Joint State Publisher GUI to interactively control the robot joints!

### Simulate in Gazebo

```bash
source ~/xarm_ws/install/setup.bash
ros2 launch xarm_description gazebo.launch.py
```

### Send Trajectory Commands

```bash
ros2 action send_goal /xarm_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [base_to_link1, link1_to_link2, link2_to_link3, \
  linkt_to_link5, link5_to_gripperbase, gripper_finger1_joint, gripper_finger2_joint], \
  points: [{positions: [0.5, 0.5, 0.5, 0.5, 0.5, -0.3, 0.3], time_from_start: {sec: 2}}]}}"
```

## 📖 Usage

For detailed usage instructions, see the [package README](src/xarm_description/README.md).

### Available Launch Files

- `display.launch.py` - RViz2 visualization with joint control
- `gazebo.launch.py` - Gazebo simulation with ros2_control
- `controller.launch.py` - Standalone controller manager

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

- **[Package README](src/xarm_description/README.md)** - Detailed usage and API
- **[Engineering Review](ENGINEERING_REVIEW.md)** - Technical architecture details
- **[Contributing Guide](CONTRIBUTING.md)** - How to contribute
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

### More Issues?

Check the [Troubleshooting Section](src/xarm_description/README.md#troubleshooting) in the package README.

## 🤝 Contributing

Contributions are welcome! Please read our [Contributing Guide](CONTRIBUTING.md) for details on our code of conduct and the process for submitting pull requests.

Areas we'd love help with:
- MoveIt2 configuration
- Hardware interface implementation
- Additional examples and tutorials
- Bug fixes and improvements

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
