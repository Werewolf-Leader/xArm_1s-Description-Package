#!/bin/bash
# XArm ROS2 Project Verification Script
# Run this after building to verify everything is set up correctly

set -e

echo "========================================="
echo "XArm ROS2 Project Verification"
echo "========================================="
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if workspace is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}✗ ROS2 not sourced${NC}"
    echo "Please run: source /opt/ros/<distro>/setup.bash"
    exit 1
else
    echo -e "${GREEN}✓ ROS2 Distribution: $ROS_DISTRO${NC}"
fi

# Check if workspace is built
if [ ! -d "install/xarm_description" ]; then
    echo -e "${RED}✗ Workspace not built${NC}"
    echo "Please run: colcon build --symlink-install"
    exit 1
else
    echo -e "${GREEN}✓ Workspace built${NC}"
fi

# Source workspace
source install/setup.bash
echo -e "${GREEN}✓ Workspace sourced${NC}"

# Check package
if ros2 pkg list | grep -q "xarm_description"; then
    echo -e "${GREEN}✓ Package 'xarm_description' found${NC}"
else
    echo -e "${RED}✗ Package 'xarm_description' not found${NC}"
    exit 1
fi

# Check launch files
echo ""
echo "Checking launch files..."
LAUNCH_FILES=("display.launch.py" "gazebo.launch.py" "controller.launch.py")
for launch in "${LAUNCH_FILES[@]}"; do
    if [ -f "src/xarm_description/launch/$launch" ]; then
        echo -e "  ${GREEN}✓${NC} $launch"
    else
        echo -e "  ${RED}✗${NC} $launch"
    fi
done

# Check URDF files
echo ""
echo "Checking URDF files..."
URDF_FILES=("xarm.xacro" "xarm.gazebo" "xarm.ros2_control.xacro" "materials.xacro")
for urdf in "${URDF_FILES[@]}"; do
    if [ -f "src/xarm_description/urdf/$urdf" ]; then
        echo -e "  ${GREEN}✓${NC} $urdf"
    else
        echo -e "  ${RED}✗${NC} $urdf"
    fi
done

# Check meshes
echo ""
echo "Checking mesh files..."
MESH_COUNT=$(find src/xarm_description/meshes -name "*.stl" 2>/dev/null | wc -l)
if [ "$MESH_COUNT" -gt 0 ]; then
    echo -e "  ${GREEN}✓${NC} Found $MESH_COUNT mesh files"
else
    echo -e "  ${YELLOW}⚠${NC} No mesh files found"
fi

# Check dependencies
echo ""
echo "Checking key dependencies..."
DEPS=("robot_state_publisher" "joint_state_publisher_gui" "gazebo_ros" "controller_manager")
for dep in "${DEPS[@]}"; do
    if ros2 pkg list | grep -q "$dep"; then
        echo -e "  ${GREEN}✓${NC} $dep"
    else
        echo -e "  ${RED}✗${NC} $dep (missing)"
    fi
done

# Test URDF (if check_urdf is available)
echo ""
if command -v check_urdf &> /dev/null; then
    echo "Validating URDF..."
    if xacro src/xarm_description/urdf/xarm.xacro > /tmp/xarm_test.urdf 2>/dev/null; then
        if check_urdf /tmp/xarm_test.urdf &> /dev/null; then
            echo -e "${GREEN}✓ URDF validation passed${NC}"
        else
            echo -e "${YELLOW}⚠ URDF validation warnings (check manually)${NC}"
        fi
        rm -f /tmp/xarm_test.urdf
    else
        echo -e "${RED}✗ Failed to process xacro${NC}"
    fi
else
    echo -e "${YELLOW}⚠ check_urdf not available (install: sudo apt install liburdfdom-tools)${NC}"
fi

# Summary
echo ""
echo "========================================="
echo "Verification Complete!"
echo "========================================="
echo ""
echo "To test the package:"
echo "  1. Visualization: ros2 launch xarm_description display.launch.py"
echo "  2. Simulation:    ros2 launch xarm_description gazebo.launch.py"
echo ""
echo "For more information, see README.md"
