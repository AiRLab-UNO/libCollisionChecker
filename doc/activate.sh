#!/bin/bash
# Convenience script to activate the Python environment for libCollisionChecker

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "Activating libCollisionChecker Python environment..."

# Activate virtual environment
source "$SCRIPT_DIR/.venv/bin/activate"

# Set PYTHONPATH
export PYTHONPATH="$SCRIPT_DIR/build:$PYTHONPATH"

echo "✓ Environment activated!"
echo ""
echo "Python module ready to use:"
echo "  import collision_checker_py as cc"
echo ""
echo "To deactivate: deactivate"
