# Installation FAQ - Common Issues

## ❌ Error: `pip install -e .` fails

**Problem:** You tried to run `pip install -e .` and got a CMake error.

**Solution:** **Don't use pip install!** This is a CMake-based project. Use the direct build method:

```bash
# If not already built:
./install_uv.sh

# If already built:
source activate.sh
```

**Why?** CMake projects don't work well with `pip install -e .` due to build system conflicts.

---

## ✅ Your Installation is Already Complete!

You successfully built the project earlier. Just activate it:

```bash
cd /home/redwan/research/2026/ResearchWorkspace/BOT-Connect/Code/libcollisionchecker
source activate.sh
python test_numpy.py
```

---

## Common Commands

### Daily Usage
```bash
# Activate environment
source activate.sh

# Use the module
python your_script.py
```

### After C++ Changes
```bash
cd build
cmake --build . -j$(nproc)
# Module automatically updated!
```

### Fresh Install
```bash
rm -rf build .venv
./install_uv.sh
```

---

## Verification Commands

```bash
# Check module loads
python -c "import collision_checker_py; print('✓ Works!')"

# Check available classes
python -c "import collision_checker_py as cc; print(dir(cc))"

# Run test
python test_numpy.py
```

---

## Environment Variables

Add to `~/.bashrc` or `~/.zshrc` for permanent setup:

```bash
export PYTHONPATH=/home/redwan/research/2026/ResearchWorkspace/BOT-Connect/Code/libcollisionchecker/build:$PYTHONPATH
```

Or use the activation script:
```bash
alias cc='source /home/redwan/research/2026/ResearchWorkspace/BOT-Connect/Code/libcollisionchecker/activate.sh'
```

Then just run: `cc`

---

## Quick Reference

| Task | Command |
|------|---------|
| Activate | `source activate.sh` |
| Test | `python test_numpy.py` |
| Rebuild | `cd build && cmake --build .` |
| Fresh install | `./install_uv.sh` |
| Deactivate | `deactivate` |

---

## Documentation Files

- **This file** - Common issues and quick fixes
- [DO_NOT_USE_PIP_INSTALL.md](DO_NOT_USE_PIP_INSTALL.md) - Why pip install doesn't work
- [QUICKSTART_UV.md](QUICKSTART_UV.md) - Quick start guide
- [INSTALL_WITH_UV.md](INSTALL_WITH_UV.md) - Detailed installation
- [INSTALLATION_SUCCESS.md](INSTALLATION_SUCCESS.md) - Post-install guide
- [examples/PYTHON_BINDINGS_README.md](examples/PYTHON_BINDINGS_README.md) - API documentation

---

## Still Having Issues?

1. **Module not found?**
   ```bash
   export PYTHONPATH=$PWD/build:$PYTHONPATH
   ```

2. **Build failed?**
   ```bash
   rm -rf build
   mkdir build && cd build
   cmake .. -DBUILD_PYTHON_BINDINGS=ON -DBUILD_TESTING=OFF
   cmake --build . -j$(nproc)
   ```

3. **Virtual environment issues?**
   ```bash
   rm -rf .venv
   uv venv
   source .venv/bin/activate
   uv pip install numpy pybind11
   ```

4. **CMake too old?**
   ```bash
   sudo snap install cmake --classic  # Ubuntu
   brew upgrade cmake                  # macOS
   ```
