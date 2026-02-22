# Quick Start: Interactive Collision Demos

## TL;DR - Run Now!

```bash
cd /home/redwan/research/2026/ResearchWorkspace/BOT-Connect/Code/libcollisionchecker
source activate.sh
python examples/interactive_collision_demo.py test/ugv/env1.yaml
```

Or use the menu:
```bash
source activate.sh
./examples/run_demo.sh
```

## What You Get

Move your mouse cursor over the environment and watch:
- 🟢 **Green robot** = Safe (no collision)
- 🔴 **Red robot** = Collision detected
- Real-time position and status display

## Available Demos

### 1. Quadtree Demo (Obstacles)
```bash
python examples/interactive_collision_demo.py test/ugv/env1.yaml
```

### 2. Occupancy Map Demo (Images)
```bash
python examples/interactive_occupancy_demo.py test/ugv/intel.yaml
```

## Examples to Try

```bash
# Simple obstacles
python examples/interactive_collision_demo.py test/ugv/env1.yaml

# Complex environment
python examples/interactive_collision_demo.py test/ugv/env3.yaml

# Intel lab map
python examples/interactive_occupancy_demo.py test/ugv/intel.yaml

# Cluttered space
python examples/interactive_occupancy_demo.py test/ugv/sr_clutter_01.yaml
```

## Screenshots

The demos show:
- Environment map (obstacles or image)
- Robot footprint (circle with radius)
- Start position (green dot)
- Goal position (red star)
- Real-time collision status

## Need Help?

See full documentation: [INTERACTIVE_DEMOS_README.md](INTERACTIVE_DEMOS_README.md)
