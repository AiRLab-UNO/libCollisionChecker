# Installing libCollisionChecker with uv

This guide covers how to install libCollisionChecker Python bindings using **uv** (the fast Python package manager).

## Prerequisites

### 1. Install uv

If you don't have uv installed yet:

```bash
# Install uv
curl -LsSf https://astral.sh/uv/install.sh | sh

# Or with pip
pip install uv
```

### 2. System Dependencies

Install the required system dependencies:

**Ubuntu/Debian:**
```bash
sudo apt update
sudo apt install -y \
    cmake \
    build-essential \
    libeigen3-dev \
    python3-dev \
    libfcl-dev \
    libccd-dev
```

**macOS:**
```bash
brew install cmake eigen fcl libccd
```

## Installation Methods

### Method 1: Direct CMake Build (Recommended)

This method gives you more control and is more reliable for C++ extensions:

```bash
cd /path/to/libcollisionchecker

# Create and activate a virtual environment with uv
uv venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate

# Install Python dependencies
uv pip install numpy pybind11

# Build with CMake
mkdir -p build && cd build
cmake .. -DBUILD_PYTHON_BINDINGS=ON \
         -DBUILD_CC_OCTREE=ON \
         -DBUILD_CC_QUAD_TREE=ON \
         -DBUILD_CC_OCCUPANCY_MAP=ON \
         -DBUILD_CC_FCL=ON \
         -DBUILD_CC_VAMP=ON \
         -DBUILD_TESTING=OFF

cmake --build . -j$(nproc)

# Add the build directory to Python path
export PYTHONPATH=$PWD:$PYTHONPATH

# Test the installation
cd ..
python -c "import collision_checker_py; print('Success!')"
```

### Method 2: Install in Development Mode with uv

```bash
cd /path/to/libcollisionchecker

# Create virtual environment
uv venv
source .venv/bin/activate

# Install in editable mode (this builds the C++ extension)
uv pip install -e .

# Or use the setup.py directly
python setup.py develop
```

### Method 3: Using uv sync (for development)

Create a `uv.lock` file by running:

```bash
cd /path/to/libcollisionchecker

# Initialize uv project (if not already done)
uv sync

# This will create a virtual environment and install dependencies
```

## Verification

After installation, verify it works:

```bash
# Check the module can be imported
python -c "import collision_checker_py as cc; print('Module version:', cc.__version__)"

# Run the example
python examples/python_example.py
```

## Common Issues and Solutions

### Issue 1: CMake Version Error with yaml-cpp

**Error:**
```
CMake Error at _deps/yaml-cpp-src/CMakeLists.txt:2 (cmake_minimum_required):
  Compatibility with CMake < 3.5 has been removed
```

**Solution:**
Update your CMake version to at least 3.22:

```bash
# Ubuntu - use Kitware's official APT repository
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | \
    gpg --dearmor - | sudo tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null
echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ focal main' | \
    sudo tee /etc/apt/sources.list.d/kitware.list >/dev/null
sudo apt update
sudo apt install cmake

# Or install from snap
sudo snap install cmake --classic

# macOS
brew upgrade cmake
```

### Issue 2: Module Not Found

**Error:**
```
ModuleNotFoundError: No module named 'collision_checker_py'
```

**Solution:**
Make sure the build directory is in your PYTHONPATH:

```bash
export PYTHONPATH=/path/to/libcollisionchecker/build:$PYTHONPATH

# Or permanently add to your shell profile
echo 'export PYTHONPATH=/path/to/libcollisionchecker/build:$PYTHONPATH' >> ~/.bashrc
source ~/.bashrc
```

### Issue 3: Missing System Dependencies

**Error:**
```
fatal error: Eigen/Dense: No such file or directory
```

**Solution:**
Install Eigen3:

```bash
sudo apt install libeigen3-dev  # Ubuntu/Debian
brew install eigen              # macOS
```

### Issue 4: FCL Not Found

**Error:**
```
CMake Error: Could not find FCL
```

**Solution:**
Either install FCL or disable it:

```bash
# Install FCL
sudo apt install libfcl-dev libccd-dev  # Ubuntu/Debian

# Or disable FCL support
cmake .. -DBUILD_CC_FCL=OFF
```

## Selecting Which Collision Checkers to Build

You can customize which collision checkers are built:

```bash
cmake .. \
    -DBUILD_PYTHON_BINDINGS=ON \
    -DBUILD_CC_OCTREE=OFF \        # Disable Octree
    -DBUILD_CC_QUAD_TREE=ON \      # Enable Quadtree
    -DBUILD_CC_OCCUPANCY_MAP=ON \  # Enable OccupancyMap
    -DBUILD_CC_FCL=OFF \           # Disable FCL
    -DBUILD_CC_VAMP=ON             # Enable VAMP
```

## Quick Start Script

Save this as `install_uv.sh`:

```bash
#!/bin/bash
set -e

# Install uv if not already installed
if ! command -v uv &> /dev/null; then
    echo "Installing uv..."
    curl -LsSf https://astral.sh/uv/install.sh | sh
fi

# Create virtual environment
echo "Creating virtual environment..."
uv venv

# Activate virtual environment
source .venv/bin/activate

# Install Python dependencies
echo "Installing Python dependencies..."
uv pip install numpy pybind11

# Build with CMake
echo "Building C++ extension..."
mkdir -p build
cd build
cmake .. -DBUILD_PYTHON_BINDINGS=ON -DBUILD_TESTING=OFF
cmake --build . -j$(nproc)

# Test installation
cd ..
export PYTHONPATH=$PWD/build:$PYTHONPATH
python -c "import collision_checker_py; print('✓ Installation successful!')"

echo ""
echo "To use the module, run:"
echo "  source .venv/bin/activate"
echo "  export PYTHONPATH=$PWD/build:\$PYTHONPATH"
```

Make it executable and run:

```bash
chmod +x install_uv.sh
./install_uv.sh
```

## Using in Your Project with uv

Add to your project's `pyproject.toml`:

```toml
[project]
dependencies = [
    "numpy>=1.19.0",
    # If libcollisionchecker is published to PyPI:
    # "libcollisionchecker>=0.1.0",
]

# For local development:
# [tool.uv.sources]
# libcollisionchecker = { path = "/path/to/libcollisionchecker", editable = true }
```

Then install with:

```bash
uv sync
```

## Environment Setup

For daily usage, add to your `.bashrc` or `.zshrc`:

```bash
# libCollisionChecker Python bindings
export COLLISION_CHECKER_ROOT="$HOME/path/to/libcollisionchecker"
export PYTHONPATH="$COLLISION_CHECKER_ROOT/build:$PYTHONPATH"
```

## Development Workflow

When developing:

```bash
# 1. Make changes to C++ code
vim src/python_bindings.cpp

# 2. Rebuild
cd build
cmake --build . -j$(nproc)

# 3. Test immediately (no reinstall needed if PYTHONPATH is set)
cd ..
python examples/python_example.py
```

## Docker Support (Optional)

Create a `Dockerfile`:

```dockerfile
FROM python:3.10-slim

# Install system dependencies
RUN apt-get update && apt-get install -y \
    cmake \
    build-essential \
    libeigen3-dev \
    libfcl-dev \
    libccd-dev \
    && rm -rf /var/lib/apt/lists/*

# Install uv
RUN pip install uv

WORKDIR /app
COPY . .

# Build
RUN uv venv && \
    . .venv/bin/activate && \
    uv pip install numpy pybind11 && \
    mkdir -p build && cd build && \
    cmake .. -DBUILD_PYTHON_BINDINGS=ON -DBUILD_TESTING=OFF && \
    cmake --build . -j$(nproc)

ENV PYTHONPATH=/app/build:$PYTHONPATH
CMD ["python", "examples/python_example.py"]
```

Build and run:

```bash
docker build -t libcollisionchecker .
docker run libcollisionchecker
```

## Next Steps

After successful installation:

1. Read the [Python Bindings README](PYTHON_BINDINGS_README.md)
2. Run the examples: `python examples/python_example.py`
3. Check the API documentation: `python -c "import collision_checker_py as cc; help(cc)"`

## Support

For issues:
1. Check this guide for common solutions
2. Verify all system dependencies are installed
3. Ensure CMake version >= 3.22
4. Check the build output for specific error messages
