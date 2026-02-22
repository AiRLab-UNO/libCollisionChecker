# Python Bindings for libCollisionChecker

This directory contains Python bindings for the libCollisionChecker library using pybind11, with full support for numpy arrays.

## Features

- **Numpy Array Support**: Pass trajectories as numpy arrays directly
- **Multiple Collision Checkers**: Bindings for all collision checker implementations
  - QuadtreeCollisionChecker (2D)
  - OctreeCollisionChecker (3D)
  - OccupancyMap (2D, image-based)
  - FCL (2D/3D with triangle meshes)
  - VampCollisionChecker (robotics-focused)
- **Easy Integration**: Simple Python API with type hints and documentation

## Requirements

- Python 3.7+
- numpy
- pybind11
- CMake 3.22+
- Eigen3

## Installation

### Build from Source

1. **Install pybind11**:
   ```bash
   pip install pybind11
   ```
   
   Or install via system package manager:
   ```bash
   # Ubuntu/Debian
   sudo apt install pybind11-dev
   
   # macOS
   brew install pybind11
   ```

2. **Build the Python module**:
   ```bash
   cd /path/to/libcollisionchecker
   mkdir -p build && cd build
   cmake .. -DBUILD_PYTHON_BINDINGS=ON
   cmake --build .
   ```

3. **Install the module** (optional):
   ```bash
   cmake --install .
   ```
   
   Or add the build directory to your Python path:
   ```bash
   export PYTHONPATH=/path/to/libcollisionchecker/build:$PYTHONPATH
   ```

### Build Options

You can control which collision checkers are built:
```bash
cmake .. \
  -DBUILD_PYTHON_BINDINGS=ON \
  -DBUILD_CC_OCTREE=ON \
  -DBUILD_CC_QUAD_TREE=ON \
  -DBUILD_CC_OCCUPANCY_MAP=ON \
  -DBUILD_CC_FCL=ON \
  -DBUILD_CC_VAMP=ON
```

## Usage

### Basic Example

```python
import numpy as np
import collision_checker_py as cc

# Load configuration
param_manager = cc.ParamManager("config.yaml")

# Create a collision checker
checker = cc.QuadtreeCollisionChecker(param_manager)

# Define a trajectory as a numpy array (N x DOF)
trajectory = np.array([
    [0.0, 0.0],  # waypoint 1
    [1.0, 1.0],  # waypoint 2
    [2.0, 2.0],  # waypoint 3
])

# Check for collision
is_collision = checker.is_collision(trajectory)
print(f"Collision detected: {is_collision}")
```

### Working with Different Dimensions

**2D Collision Checking** (QuadtreeCollisionChecker, OccupancyMap):
```python
# 2D trajectory: shape (N, 2)
trajectory_2d = np.array([
    [x1, y1],
    [x2, y2],
    [x3, y3],
])
```

**3D Collision Checking** (OctreeCollisionChecker, FCL):
```python
# 3D trajectory: shape (N, 3)
trajectory_3d = np.array([
    [x1, y1, z1],
    [x2, y2, z2],
    [x3, y3, z3],
])
```

**Robot Joint Space** (VampCollisionChecker):
```python
# Joint angles: shape (N, num_joints)
trajectory_joints = np.array([
    [q1_1, q2_1, q3_1, q4_1, q5_1, q6_1],  # first configuration
    [q1_2, q2_2, q3_2, q4_2, q5_2, q6_2],  # second configuration
])
```

### Advanced Usage

**Generating Smooth Trajectories**:
```python
import numpy as np

# Generate a circular trajectory
num_waypoints = 100
t = np.linspace(0, 2*np.pi, num_waypoints)
trajectory = np.column_stack([
    np.cos(t) * 5.0,  # x coordinates
    np.sin(t) * 5.0   # y coordinates
])

is_collision = checker.is_collision(trajectory)
```

**Batch Collision Checking**:
```python
# Check multiple trajectories
trajectories = [traj1, traj2, traj3]
results = [checker.is_collision(traj) for traj in trajectories]
collision_free = [not result for result in results]
```

### Available Classes

#### Collision Checkers

- **`QuadtreeCollisionChecker(param_manager)`**
  - 2D collision checking using quadtree spatial decomposition
  - Fast for 2D environments

- **`OctreeCollisionChecker(param_manager)`**
  - 3D collision checking using octree spatial decomposition
  - Efficient for 3D environments

- **`OccupancyMap(param_manager)`**
  - 2D image-based collision checking
  - Works with PNG/image files

- **`FCL(param_manager)`**
  - Flexible Collision Library support
  - Supports triangle meshes and 2D/3D primitives

- **`VampCollisionChecker(param_manager)`**
  - Optimized for robot motion planning
  - Vectorized collision checking

#### Utility Classes

- **`ParamManager(yaml_file)`**
  - Loads configuration from YAML file
  - Required for initializing collision checkers

#### Methods

All collision checkers inherit from `BaseCollisionChecker` and provide:

- **`is_collision(trajectory: np.ndarray) -> bool`**
  - Check if trajectory collides
  - Input: numpy array of shape `(N, DOF)` where N is the number of waypoints
  - Returns: `True` if collision detected, `False` otherwise

- **`is_collision_vector(trajectory: List[Eigen::VectorXd]) -> bool`**
  - Advanced: Direct C++ vector interface (rarely needed)

### Utility Functions

- **`numpy_to_trajectory(array: np.ndarray) -> List[Eigen::VectorXd]`**
  - Convert numpy array to C++ vector format

- **`trajectory_to_numpy(trajectory: List[Eigen::VectorXd]) -> np.ndarray`**
  - Convert C++ vector format to numpy array

## Examples

See [python_example.py](python_example.py) for complete examples of all collision checkers.

Run the examples:
```bash
python python_example.py
```

## Configuration

Each collision checker requires a YAML configuration file. Example structure:

```yaml
collision_checker:
  type: "quadtree"  # or "octree", "occupancy", "fcl", "vamp"
  
  # Quadtree/Octree specific
  resolution: 0.1
  bounds:
    min: [-10.0, -10.0]
    max: [10.0, 10.0]
  
  # Occupancy map specific
  image_file: "map.png"
  resolution: 0.05
  
  # FCL specific
  mesh_file: "robot.stl"
  
  # VAMP specific
  robot_name: "panda"
  urdf_file: "panda.urdf"
```

## Troubleshooting

### Module not found
```python
ModuleNotFoundError: No module named 'collision_checker_py'
```
**Solution**: Add the build directory to your Python path:
```bash
export PYTHONPATH=/path/to/libcollisionchecker/build:$PYTHONPATH
```

### Shape mismatch errors
```
RuntimeError: Trajectory must be a 2D numpy array (N x DOF)
```
**Solution**: Ensure your trajectory is a 2D numpy array with shape `(num_waypoints, dof)`:
```python
# Wrong: 1D array
trajectory = np.array([1, 2, 3, 4])

# Correct: 2D array
trajectory = np.array([[1, 2], [3, 4]])  # 2 waypoints, 2 DOF
```

### Build errors with pybind11
**Solution**: Make sure pybind11 is properly installed and can be found by CMake:
```bash
pip install pybind11
# or
sudo apt install pybind11-dev
```

## Performance Tips

1. **Use contiguous arrays**: Ensure numpy arrays are C-contiguous for best performance
   ```python
   trajectory = np.ascontiguousarray(trajectory)
   ```

2. **Batch operations**: Check multiple trajectories in a loop rather than one at a time

3. **Choose appropriate checker**: Use 2D checkers (Quadtree, OccupancyMap) for 2D problems for better performance

## API Reference

For detailed API documentation, use Python's help system:
```python
import collision_checker_py as cc
help(cc)
help(cc.QuadtreeCollisionChecker)
help(cc.BaseCollisionChecker.is_collision)
```

## License

Same as libCollisionChecker parent project.

## Contributing

Contributions are welcome! Please ensure:
- All collision checker types are supported
- Numpy array handling is efficient
- Documentation is updated
- Examples are provided

## Support

For issues, questions, or contributions, please visit the GitHub repository.
