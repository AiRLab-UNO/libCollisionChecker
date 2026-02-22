# ⚠️ IMPORTANT: Installation Method

## DO NOT use `pip install -e .`

For C++ projects with CMake, **pip install does not work properly**. Use the CMake method instead.

## ✅ Correct Installation Method

### Option 1: Automated Script (Easiest)
```bash
./install_uv.sh
```

### Option 2: Manual Installation
```bash
# 1. Create virtual environment and install dependencies
uv venv
source .venv/bin/activate
uv pip install numpy pybind11

# 2. Build with CMake
mkdir -p build && cd build
cmake .. -DBUILD_PYTHON_BINDINGS=ON -DBUILD_TESTING=OFF
cmake --build . -j$(nproc)
cd ..

# 3. Set Python path
export PYTHONPATH=$PWD/build:$PYTHONPATH

# 4. Test
python test_numpy.py
```

### Option 3: Quick Activation (If Already Built)
```bash
source activate.sh
```

## Why Not pip install?

The `pip install -e .` command tries to use setuptools to build CMake projects, which:
- ❌ Has configuration conflicts
- ❌ Doesn't handle CMake dependencies well
- ❌ Creates temporary build directories that cause issues

The **direct CMake build** approach:
- ✅ Uses the native CMake build system
- ✅ Provides better control over build options
- ✅ Works reliably with complex C++ dependencies
- ✅ Is the standard for CMake-based Python extensions

## Already Built?

Your installation is already complete! Just activate:

```bash
source .venv/bin/activate
export PYTHONPATH=/home/redwan/research/2026/ResearchWorkspace/BOT-Connect/Code/libcollisionchecker/build:$PYTHONPATH
python test_numpy.py
```

## For More Info

- Quick Start: [QUICKSTART_UV.md](QUICKSTART_UV.md)
- Full Guide: [INSTALL_WITH_UV.md](INSTALL_WITH_UV.md)
- Success Guide: [INSTALLATION_SUCCESS.md](INSTALLATION_SUCCESS.md)
