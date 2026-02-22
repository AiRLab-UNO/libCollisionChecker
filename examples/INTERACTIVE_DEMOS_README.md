# Interactive Collision Checker Demos

This directory contains interactive matplotlib-based demos for visualizing collision checking in real-time.

## Available Demos

### 1. Interactive Collision Demo (Quadtree)
**File**: `interactive_collision_demo.py`

Interactive visualization using Quadtree collision checker with obstacle-based environments.

**Usage:**
```bash
python examples/interactive_collision_demo.py test/ugv/env1.yaml
```

**Features:**
- Real-time collision checking as you move the mouse
- Visualizes obstacles, start, and goal positions
- Green robot = Safe, Red robot = Collision
- Displays current position and collision status

**Works with:**
- `test/ugv/env1.yaml`
- `test/ugv/env2.yaml`
- `test/ugv/env3.yaml`
- Any YAML file with obstacles defined

---

### 2. Interactive Occupancy Demo (Image-based)
**File**: `interactive_occupancy_demo.py`

Interactive visualization using Occupancy Map collision checker with image-based maps.

**Usage:**
```bash
python examples/interactive_occupancy_demo.py test/ugv/intel.yaml
python examples/interactive_occupancy_demo.py test/ugv/sr_clutter_01.yaml
```

**Features:**
- Displays PNG occupancy map as background
- Real-time collision checking on the map
- Shows free space (white) vs obstacles (black)
- Visualizes robot footprint with radius

**Works with:**
- `test/ugv/intel.yaml` (with `intel_map.png`)
- `test/ugv/sr_clutter_01.yaml` (with `sr_clutter_01.png`)
- `test/ugv/sr_clutter_04.yaml` (with `sr_clutter_04.png`)
- `test/ugv/sr_nonconvex_02.yaml` (with `sr_nonconvex_02.png`)

---

## Installation

Make sure you have the required dependencies:

```bash
source activate.sh
uv pip install matplotlib pyyaml
```

Or if already in requirements:
```bash
pip install matplotlib pyyaml
```

---

## Quick Start

### Run Quadtree Demo
```bash
cd /path/to/libcollisionchecker
source activate.sh
python examples/interactive_collision_demo.py test/ugv/env1.yaml
```

### Run Occupancy Map Demo
```bash
cd /path/to/libcollisionchecker
source activate.sh
python examples/interactive_occupancy_demo.py test/ugv/intel.yaml
```

---

## Controls

**Both demos:**
- Move your mouse cursor over the plot area
- The robot (circle) follows your cursor
- **Green robot** = No collision detected (safe)
- **Red robot** = Collision detected
- Current position displayed in top-left corner
- Close the window to exit

---

## Environment Files

### Quadtree Environments (obstacles)
Located in `test/ugv/`:
- `env1.yaml` - Simple obstacles
- `env2.yaml` - Medium complexity
- `env3.yaml` - Complex layout
- `env4.yaml`, `env5.yaml` - Various layouts
- `env_bug_trap.yaml` - Bug trap scenario
- `env_nonconvex.yaml` - Non-convex obstacles

### Occupancy Map Environments (images)
Located in `test/ugv/`:
- `intel.yaml` + `intel_map.png` - Intel research lab
- `sr_clutter_01.yaml` + `sr_clutter_01.png` - Cluttered space
- `sr_clutter_04.yaml` + `sr_clutter_04.png` - Another cluttered space
- `sr_nonconvex_02.yaml` + `sr_nonconvex_02.png` - Non-convex space

---

## Creating Your Own Environment

### For Quadtree (Obstacles)

Create a YAML file with:
```yaml
robot_radius: 0.345
obstacle_length: 0.25
boundary: [-20.0, 1.0, -20.0, 2.0]  # [x_min, x_max, y_min, y_max]
start: [-10.0, -2.0, 1.5707]  # [x, y, theta]
goal: [0.0, 0.0]
collision_checker: olc  # or quadtree

obstacles: [
    [-10.0, -8.0],
    [-6.0, -8.0],
    # Add more [x, y] positions
]
```

### For Occupancy Map (Image)

1. Create a PNG image (white = free space, black = obstacles)
2. Create a YAML file:
```yaml
robot_radius: 0.345
resolution: 0.05  # meters per pixel
origin: [-10.0, -10.0]  # [x, y] of image origin
start: [5.0, 5.0, 0.0]
goal: [15.0, 15.0]
collision_checker: occupancy
```

3. Name them with matching names (e.g., `mymap.yaml` and `mymap.png`)

---

## Example Output

When running the demos, you'll see:
```
======================================================================
Interactive Collision Checker Demo
======================================================================
Loading environment: test/ugv/env1.yaml

Controls:
  - Move mouse cursor to check collision at that position
  - Green robot = Safe (no collision)
  - Red robot = Collision detected

Close the window to exit.
======================================================================
```

Then a matplotlib window opens showing the environment with interactive collision checking.

---

## Troubleshooting

### "Module not found: matplotlib"
```bash
uv pip install matplotlib
```

### "Module not found: yaml"
```bash
uv pip install pyyaml
```

### "Module not found: collision_checker_py"
```bash
source activate.sh
```

### "File not found: test/ugv/env1.yaml"
Make sure you're running from the libcollisionchecker root directory:
```bash
cd /path/to/libcollisionchecker
python examples/interactive_collision_demo.py test/ugv/env1.yaml
```

### Window doesn't open
- Make sure you have a display available (not running in headless SSH)
- Try using X11 forwarding if on remote system: `ssh -X user@host`

---

## Tips

1. **Test different environments**: Try all the YAML files in `test/ugv/`
2. **Modify robot radius**: Edit the YAML file to change robot size
3. **Add obstacles**: Edit YAML to add more obstacles
4. **Create custom maps**: Use image editing software to create PNG occupancy maps
5. **Compare checkers**: Use quadtree for obstacles, occupancy for image-based maps

---

## For Developers

### Adding New Features

Both demos follow a similar structure:
- `__init__`: Load YAML and create collision checker
- `setup_plot`: Create matplotlib figure
- `on_mouse_move`: Handle mouse events
- `check_collision`: Call collision checker with current position
- `update_robot_visualization`: Update colors based on result

To add features:
1. Modify the respective demo file
2. Add new visualization elements in `setup_plot`
3. Update `on_mouse_move` or `check_collision` for new functionality

### Extending to 3D

For 3D collision checking (Octree):
- Use `mpl_toolkits.mplot3d` for 3D visualization
- Track mouse position + a slider for Z coordinate
- Replace QuadtreeCollisionChecker with OctreeCollisionChecker

---

## Additional Examples

Other examples in this directory:
- `python_example.py` - Complete API examples for all collision checkers
- `test_numpy.py` - Quick numpy array test (in parent directory)

---

**Enjoy interactive collision checking! 🎉**
