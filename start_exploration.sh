#!/bin/bash

# Quick start script for multi-drone area exploration

echo "============================================================"
echo "   Multi-Drone Area Exploration - Quick Start"
echo "============================================================"
echo ""
echo "Mission Configuration:"
echo "  • 6 Explorer Drones (Drone 0-5)"
echo "    - Each assigned to a specific area (1-6)"
echo "    - Will systematically explore entire area"
echo ""
echo "  • 2 Backup Drones (Drone 6-7)"
echo "    - Remain at starting position"
echo "    - Ready for emergency deployment"
echo ""
echo "Area Assignments:"
echo "  Drone 0 → Area 1 (Red)    at (-10, 10)"
echo "  Drone 1 → Area 2 (Blue)   at (0, 10)"
echo "  Drone 2 → Area 3 (Yellow) at (10, 10)"
echo "  Drone 3 → Area 4 (Purple) at (-10, 0)"
echo "  Drone 4 → Area 5 (Orange) at (0, 0)"
echo "  Drone 5 → Area 6 (Cyan)   at (10, 0)"
echo "  Drone 6 → BACKUP at start position"
echo "  Drone 7 → BACKUP at start position"
echo ""
echo "============================================================"
echo ""
echo "Starting exploration mission..."
echo "Press Ctrl+C to stop"
echo ""

# Source the workspace
source /home/sachin/catkin_ws/devel/setup.bash

# Launch the exploration simulation
roslaunch multi_drone_sim explore_areas.launch
