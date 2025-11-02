#!/bin/bash

# Check if the multi-drone simulation environment is set up correctly

echo "=========================================="
echo "Multi-Drone Simulation - System Check"
echo "=========================================="
echo ""

# Check ROS installation
echo "1. Checking ROS installation..."
if [ -z "$ROS_DISTRO" ]; then
    echo "   ❌ ROS not sourced. Run: source /opt/ros/noetic/setup.bash"
else
    echo "   ✓ ROS $ROS_DISTRO detected"
fi
echo ""

# Check Gazebo
echo "2. Checking Gazebo..."
if command -v gazebo &> /dev/null; then
    GAZEBO_VERSION=$(gazebo --version | head -n1)
    echo "   ✓ $GAZEBO_VERSION"
else
    echo "   ❌ Gazebo not found"
fi
echo ""

# Check Python 3
echo "3. Checking Python 3..."
if command -v python3 &> /dev/null; then
    PYTHON_VERSION=$(python3 --version)
    echo "   ✓ $PYTHON_VERSION"
else
    echo "   ❌ Python 3 not found"
fi
echo ""

# Check PyYAML
echo "4. Checking PyYAML..."
if python3 -c "import yaml" 2>/dev/null; then
    echo "   ✓ PyYAML installed"
else
    echo "   ❌ PyYAML not installed. Run: pip3 install pyyaml"
fi
echo ""

# Check workspace
echo "5. Checking catkin workspace..."
if [ -f "/home/sachin/catkin_ws/devel/setup.bash" ]; then
    echo "   ✓ Workspace built"
else
    echo "   ❌ Workspace not built. Run: cd ~/catkin_ws && catkin_make"
fi
echo ""

# Check package files
echo "6. Checking package files..."
PACKAGE_PATH="/home/sachin/catkin_ws/src/multi_drone_sim"

if [ -d "$PACKAGE_PATH" ]; then
    echo "   ✓ Package directory exists"
    
    # Check key files
    FILES=(
        "worlds/field_areas.world"
        "models/quadcopter/model.sdf"
        "launch/multi_drone_sim.launch"
        "scripts/multi_drone_navigator.py"
        "config/areas.yaml"
    )
    
    for file in "${FILES[@]}"; do
        if [ -f "$PACKAGE_PATH/$file" ]; then
            echo "   ✓ $file"
        else
            echo "   ❌ $file missing"
        fi
    done
else
    echo "   ❌ Package directory not found"
fi
echo ""

# Check ROS dependencies
echo "7. Checking ROS dependencies..."
DEPS=("gazebo_ros" "gazebo_plugins" "geometry_msgs" "nav_msgs")
for dep in "${DEPS[@]}"; do
    if rospack find "$dep" &> /dev/null; then
        echo "   ✓ $dep"
    else
        echo "   ❌ $dep not found"
    fi
done
echo ""

echo "=========================================="
echo "System check complete!"
echo "=========================================="
echo ""
echo "If all checks passed, run:"
echo "  source ~/catkin_ws/devel/setup.bash"
echo "  roslaunch multi_drone_sim multi_drone_sim.launch"
echo ""
