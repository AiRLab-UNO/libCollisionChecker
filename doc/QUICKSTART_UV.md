# Quick Installation Guide for uv

## TL;DR - Fastest Way

```bash
cd /path/to/libcollisionchecker
./install_uv.sh
```

That's it! The script handles everything automatically.

---

## Manual Installation (3 Steps)

### Step 1: Setup Environment
```bash
uv venv
source .venv/bin/activate
uv pip install numpy pybind11
```

### Step 2: Build
```bash
mkdir -p build && cd build
cmake .. -DBUILD_PYTHON_BINDINGS=ON -DBUILD_TESTING=OFF
cmake --build . -j$(nproc)
```

### Step 3: Configure Python Path
```bash
export PYTHONPATH=$PWD:$PYTHONPATH
```

---

## Daily Usage

```bash
# Activate environment
source .venv/bin/activate

# Set path (or add to ~/.bashrc)
export PYTHONPATH=/path/to/libcollisionchecker/build:$PYTHONPATH

# Use it
python your_script.py
```

---

## Test Installation

```bash
python -c "import collision_checker_py; print('✓ Works!')"
python examples/python_example.py
```

---

## Common Issues

### "Module not found"
```bash
export PYTHONPATH=/path/to/libcollisionchecker/build:$PYTHONPATH
```

### "CMake too old"
Need CMake >= 3.22. Upgrade:
```bash
sudo snap install cmake --classic  # Ubuntu
brew upgrade cmake                  # macOS
```

### "FCL not found"
Either install it or disable:
```bash
sudo apt install libfcl-dev libccd-dev  # Install
# OR
cmake .. -DBUILD_CC_FCL=OFF             # Disable
```

---

## Example Code

```python
import numpy as np
import collision_checker_py as cc

# Load config and create checker
pm = cc.ParamManager("config.yaml")
checker = cc.QuadtreeCollisionChecker(pm)

# Check trajectory (numpy array)
trajectory = np.array([[0, 0], [1, 1], [2, 2]])
is_collision = checker.is_collision(trajectory)
print(f"Collision: {is_collision}")
```

---

## Need More Help?

See detailed guide: [INSTALL_WITH_UV.md](INSTALL_WITH_UV.md)
