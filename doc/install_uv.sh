#!/bin/bash
# Installation script for libCollisionChecker with uv
set -e

echo "================================================"
echo "libCollisionChecker Installation with uv"
echo "================================================"
echo ""

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored messages
print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1"
}

print_info() {
    echo -e "${YELLOW}➜${NC} $1"
}

# Check if uv is installed
if ! command -v uv &> /dev/null; then
    print_info "uv not found. Installing uv..."
    curl -LsSf https://astral.sh/uv/install.sh | sh
    export PATH="$HOME/.cargo/bin:$PATH"
    print_success "uv installed"
else
    print_success "uv is already installed"
fi

# Check CMake version
print_info "Checking CMake version..."
if command -v cmake &> /dev/null; then
    CMAKE_VERSION=$(cmake --version | head -n1 | cut -d' ' -f3)
    REQUIRED_VERSION="3.22"
    if [ "$(printf '%s\n' "$REQUIRED_VERSION" "$CMAKE_VERSION" | sort -V | head -n1)" = "$REQUIRED_VERSION" ]; then
        print_success "CMake $CMAKE_VERSION is installed"
    else
        print_error "CMake version $CMAKE_VERSION is too old. Need >= 3.22"
        echo "Please upgrade CMake. See INSTALL_WITH_UV.md for instructions."
        exit 1
    fi
else
    print_error "CMake is not installed"
    echo "Please install CMake >= 3.22. See INSTALL_WITH_UV.md for instructions."
    exit 1
fi

# Check for Eigen3
print_info "Checking for Eigen3..."
if pkg-config --exists eigen3 2>/dev/null || [ -d "/usr/include/eigen3" ] || [ -d "/usr/local/include/eigen3" ]; then
    print_success "Eigen3 is installed"
else
    print_error "Eigen3 not found"
    echo "Install with: sudo apt install libeigen3-dev (Ubuntu) or brew install eigen (macOS)"
    exit 1
fi

# Create virtual environment
print_info "Creating virtual environment with uv..."
uv venv
print_success "Virtual environment created"

# Activate virtual environment
print_info "Activating virtual environment..."
source .venv/bin/activate
print_success "Virtual environment activated"

# Install Python dependencies
print_info "Installing Python dependencies..."
uv pip install numpy pybind11
print_success "Python dependencies installed"

# Create build directory
print_info "Creating build directory..."
mkdir -p build
cd build

# Configure with CMake
print_info "Configuring with CMake..."
cmake .. \
    -DBUILD_PYTHON_BINDINGS=ON \
    -DBUILD_CC_OCTREE=ON \
    -DBUILD_CC_QUAD_TREE=ON \
    -DBUILD_CC_OCCUPANCY_MAP=ON \
    -DBUILD_CC_FCL=ON \
    -DBUILD_CC_VAMP=ON \
    -DBUILD_TESTING=OFF

if [ $? -eq 0 ]; then
    print_success "CMake configuration successful"
else
    print_error "CMake configuration failed"
    echo "Check the error messages above. You may need to disable some features."
    echo "For example, if FCL is not installed, use: -DBUILD_CC_FCL=OFF"
    exit 1
fi

# Build
print_info "Building C++ extension (this may take a few minutes)..."
NUM_CORES=$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)
cmake --build . -j${NUM_CORES}

if [ $? -eq 0 ]; then
    print_success "Build successful"
else
    print_error "Build failed"
    exit 1
fi

# Test installation
cd ..
export PYTHONPATH=$PWD/build:$PYTHONPATH

print_info "Testing installation..."
python -c "import collision_checker_py as cc; print('Module loaded successfully'); print('Version:', cc.__version__)"

if [ $? -eq 0 ]; then
    print_success "Installation test passed"
else
    print_error "Installation test failed"
    exit 1
fi

echo ""
echo "================================================"
print_success "Installation completed successfully!"
echo "================================================"
echo ""
echo "To use the module:"
echo "  1. Activate the virtual environment:"
echo "     ${GREEN}source .venv/bin/activate${NC}"
echo ""
echo "  2. Set the PYTHONPATH:"
echo "     ${GREEN}export PYTHONPATH=$PWD/build:\$PYTHONPATH${NC}"
echo ""
echo "  3. Test with examples:"
echo "     ${GREEN}python examples/python_example.py${NC}"
echo ""
echo "  4. Use in your code:"
echo "     ${GREEN}import collision_checker_py as cc${NC}"
echo ""
echo "Add this to your ~/.bashrc or ~/.zshrc for permanent setup:"
echo "  ${YELLOW}export PYTHONPATH=$PWD/build:\$PYTHONPATH${NC}"
echo ""
