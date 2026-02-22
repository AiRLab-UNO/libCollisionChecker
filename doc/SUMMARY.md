# libCollisionChecker Python Bindings - Summary

## ✅ Status: INSTALLED & WORKING

Your Python bindings with numpy array support are successfully installed!

---

## How to Use (2 Simple Steps)

### 1. Activate
```bash
source activate.sh
```

### 2. Import and Use
```python
import collision_checker_py as cc
```

That's it! 🎉

---

## What You Have

✅ **Module Name**: `collision_checker_py`  
✅ **Version**: 1.0.0  
✅ **Numpy Support**: Full support for numpy arrays  
✅ **Location**: `./build/`  
✅ **Virtual Environment**: `.venv/`

### Available Collision Checkers

1. **QuadtreeCollisionChecker** - 2D spatial decomposition
2. **OctreeCollisionChecker** - 3D spatial decomposition  
3. **OccupancyMap** - 2D image-based
4. **FCL** - 2D/3D with triangle meshes
5. **VampCollisionChecker** - Robotics-optimized

---

## Installation Method Used

✅ **CMake Direct Build** (Recommended for C++ projects)

```bash
uv venv                              # Create virtual environment
uv pip install numpy pybind11        # Install dependencies
cmake .. -DBUILD_PYTHON_BINDINGS=ON  # Configure
cmake --build .                      # Build
```

❌ **NOT pip install** (doesn't work with CMake)

---

## Daily Workflow

### Morning Setup
```bash
cd /path/to/libcollisionchecker
source activate.sh
```

### Use the Module
```python
import numpy as np
import collision_checker_py as cc

trajectory = np.array([[0, 0], [1, 1]])
# Use with collision checkers...
```

### After C++ Changes
```bash
cd build && cmake --build . -j$(nproc)
# No reinstall needed!
```

---

## Documentation Index

| File | Purpose |
|------|---------|
| [PYTHON_BINDINGS.md](PYTHON_BINDINGS.md) | Quick reference (start here) |
| [FAQ.md](FAQ.md) | Common issues & solutions |
| [QUICKSTART_UV.md](QUICKSTART_UV.md) | Installation TL;DR |
| [INSTALL_WITH_UV.md](INSTALL_WITH_UV.md) | Detailed installation guide |
| [DO_NOT_USE_PIP_INSTALL.md](DO_NOT_USE_PIP_INSTALL.md) | Why pip install fails |
| [examples/PYTHON_BINDINGS_README.md](examples/PYTHON_BINDINGS_README.md) | Full API documentation |
| [examples/python_example.py](examples/python_example.py) | Complete examples |
| [test_numpy.py](test_numpy.py) | Quick numpy test |

---

## Quick Commands

```bash
# Activate environment
source activate.sh

# Test installation
python test_numpy.py

# Check version
python -c "import collision_checker_py as cc; print(cc.__version__)"

# See available classes
python -c "import collision_checker_py as cc; help(cc)"

# Rebuild after changes
cd build && cmake --build . -j$(nproc)
```

---

## Permanent Setup (Optional)

Add to `~/.bashrc` or `~/.zshrc`:

```bash
# libCollisionChecker
export PYTHONPATH=/home/redwan/research/2026/ResearchWorkspace/BOT-Connect/Code/libcollisionchecker/build:$PYTHONPATH
alias cc='source /home/redwan/research/2026/ResearchWorkspace/BOT-Connect/Code/libcollisionchecker/activate.sh'
```

Then just run: `cc`

---

## Example Code

```python
import numpy as np
import collision_checker_py as cc

# Create numpy trajectory
trajectory = np.array([
    [0.0, 0.0],  # waypoint 1
    [1.0, 1.0],  # waypoint 2
    [2.0, 2.0],  # waypoint 3
])

# Load config and check collision
pm = cc.ParamManager("config.yaml")
checker = cc.QuadtreeCollisionChecker(pm)
is_collision = checker.is_collision(trajectory)

print(f"Collision detected: {is_collision}")
```

---

## Troubleshooting

### "Module not found"
```bash
export PYTHONPATH=$PWD/build:$PYTHONPATH
```

### "pip install doesn't work"
Don't use pip install! See [DO_NOT_USE_PIP_INSTALL.md](DO_NOT_USE_PIP_INSTALL.md)

### Need to reinstall?
```bash
./install_uv.sh
```

### More issues?
Check [FAQ.md](FAQ.md)

---

## What's Next?

1. ✅ Installation complete
2. 📝 Create your `config.yaml` file
3. 🎯 Set up your environment (obstacles, maps)
4. 🚀 Start checking trajectories!

---

**You're all set! Happy collision checking! 🎉**
