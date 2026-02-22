#!/bin/bash
# Quick launcher for interactive collision demos

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

echo "=========================================="
echo "Interactive Collision Checker Demos"
echo "=========================================="
echo ""
echo "Select a demo to run:"
echo ""
echo "Quadtree (Obstacle-based) Demos:"
echo "  1) env1.yaml - Simple obstacles"
echo "  2) env2.yaml - Medium complexity"
echo "  3) env3.yaml - Complex layout"
echo "  4) env_bug_trap.yaml - Bug trap"
echo "  5) env_nonconvex.yaml - Non-convex"
echo "  5a) env_rectangles.yaml - Rectangular obstacles"
echo ""
echo "Occupancy Map (Image-based) Demos:"
echo "  6) intel.yaml - Intel lab map"
echo "  7) sr_clutter_01.yaml - Cluttered space"
echo "  8) sr_nonconvex_02.yaml - Non-convex space"
echo ""
echo "Trajectory Demos (Click to create path):"
echo "  9) Trajectory demo - env1.yaml"
echo ""
echo "  0) Exit"
echo ""
read -p "Enter your choice (0-9, 5a): " choice

case $choice in
    1)
        python "$SCRIPT_DIR/interactive_collision_demo.py" "$REPO_ROOT/test/ugv/env1.yaml"
        ;;
    2)
        python "$SCRIPT_DIR/interactive_collision_demo.py" "$REPO_ROOT/test/ugv/env2.yaml"
        ;;
    3)
        python "$SCRIPT_DIR/interactive_collision_demo.py" "$REPO_ROOT/test/ugv/env3.yaml"
        ;;
    4)
        python "$SCRIPT_DIR/interactive_collision_demo.py" "$REPO_ROOT/test/ugv/env_bug_trap.yaml"
        ;;
    5)
        python "$SCRIPT_DIR/interactive_collision_demo.py" "$REPO_ROOT/test/ugv/env_nonconvex.yaml"
        ;;
    5a)
        python "$SCRIPT_DIR/interactive_collision_demo.py" "$REPO_ROOT/test/ugv/env_rectangles.yaml"
        ;;
    6)
        python "$SCRIPT_DIR/interactive_occupancy_demo.py" "$REPO_ROOT/test/ugv/intel.yaml"
        ;;
    7)
        python "$SCRIPT_DIR/interactive_occupancy_demo.py" "$REPO_ROOT/test/ugv/sr_clutter_01.yaml"
        ;;
    8)
        python "$SCRIPT_DIR/interactive_occupancy_demo.py" "$REPO_ROOT/test/ugv/sr_nonconvex_02.yaml"
        ;;
    9)
        python "$SCRIPT_DIR/interactive_trajectory_demo.py" "$REPO_ROOT/test/ugv/env1.yaml"
        ;;
    0)
        echo "Exiting..."
        exit 0
        ;;
    *)
        echo "Invalid choice. Exiting."
        exit 1
        ;;
esac
