# Python Bindings - Quick Reference

## ⚡ Installation Already Complete!

Your Python bindings are already built and ready to use.

## 🚀 Quick Start

```bash
# Activate the environment
source activate.sh

# Test it works
python test_numpy.py

# Use in your code
python -c "import collision_checker_py as cc; print(cc.__version__)"
```

## 📖 Documentation

- **[FAQ.md](FAQ.md)** ← Start here for common issues
- [QUICKSTART_UV.md](QUICKSTART_UV.md) - Quick installation guide
- [INSTALL_WITH_UV.md](INSTALL_WITH_UV.md) - Detailed installation
- [examples/PYTHON_BINDINGS_README.md](examples/PYTHON_BINDINGS_README.md) - Full API docs

## ⚠️ Important Notes

- ❌ **Don't use** `pip install -e .` (it doesn't work for CMake projects)
- ✅ **Do use** `source activate.sh` (already built and ready)
- 📝 See [DO_NOT_USE_PIP_INSTALL.md](DO_NOT_USE_PIP_INSTALL.md) for explanation

## 💡 Example Usage

```python
import numpy as np
import collision_checker_py as cc

# Create trajectory (numpy array)
trajectory = np.array([
    [0.0, 0.0],
    [1.0, 1.0],
    [2.0, 2.0],
])

# Check collision (requires config.yaml)
pm = cc.ParamManager("config.yaml")
checker = cc.QuadtreeCollisionChecker(pm)
is_collision = checker.is_collision(trajectory)
print(f"Collision: {is_collision}")
```

## 🔧 Rebuilding

If you modify C++ code:

```bash
cd build
cmake --build . -j$(nproc)
# Module automatically updated!
```

## 📞 Need Help?

Check [FAQ.md](FAQ.md) for solutions to common problems.
