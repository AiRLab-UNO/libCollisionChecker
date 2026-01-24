# Collision Checker Tests

This directory contains Google Test-based unit tests for the collision checker modules.

## Test Structure

Each collision checker module has its own test file:

- `test_octree_collision_checker.cpp` - Tests for 3D Octree collision detection (UAV scenarios)
- `test_quadtree_collision_checker.cpp` - Tests for 2D Quadtree collision detection (UGV scenarios)
- `test_occupancy_map.cpp` - Tests for occupancy grid map-based collision detection
- `test_fcl_collision_checker.cpp` - Placeholder tests for FCL collision checker (when enabled)

## Test Data

Tests use YAML configuration files from the `test/` directory:

### UAV Test Files (3D - Octree)
- `test/uav/quad1.yaml` through `quad5.yaml` - Various 3D environments with cubic obstacles

### UGV Test Files (2D - Quadtree and Occupancy Map)
- `test/ugv/env1.yaml` through `env5.yaml` - 2D environments with point obstacles
- `test/ugv/env_bug_trap.yaml`, `env_nonconvex.yaml` - Special scenario environments
- `test/ugv/intel.yaml`, `sr_clutter_*.yaml`, `sr_nonconvex_*.yaml` - Map-based environments with PNG images

## Building and Running Tests

### Prerequisites
- Google Test library installed
- All collision checker dependencies (Eigen3, yaml-cpp, etc.)

### Build Tests
```bash
cd build
cmake .. -DBUILD_TESTING=ON
cmake --build .
```

### Run All Tests
```bash
# From build directory
ctest --output-on-failure

# Or use the custom target
make run_all_tests
```

### Run Individual Test Suites
```bash
# Run only Octree tests
./tests/octree_collision_checker_test

# Run only Quadtree tests
./tests/quadtree_collision_checker_test

# Run only Occupancy Map tests
./tests/occupancy_map_test
```

### Run with Verbose Output
```bash
./tests/octree_collision_checker_test --gtest_filter="*" --gtest_color=yes
```

## Test Coverage

Each test suite covers:

### Functional Tests
- ✅ Constructor and initialization
- ✅ Loading multiple configuration files
- ✅ Collision detection at specific positions
- ✅ Trajectory collision checking
- ✅ Boundary checking
- ✅ Empty and single-point trajectories
- ✅ Long trajectory handling
- ✅ Edge cases (near misses, exact collisions)

### Performance Tests
- ✅ Bulk collision checking (1000+ queries)
- ✅ Performance benchmarking

### Robustness Tests
- ✅ Invalid input handling
- ✅ Dimension handling (2D, 3D, extra dimensions)
- ✅ Reverse trajectories
- ✅ Dense sampling

## Test Configuration

Tests are enabled/disabled based on the build options in CMakeLists.txt:

- `BUILD_CC_OCTREE=ON` → Octree tests enabled
- `BUILD_CC_QUAD_TREE=ON` → Quadtree tests enabled
- `BUILD_CC_OCCUPANCY_MAP=ON` → Occupancy Map tests enabled
- `BUILD_CC_FCL=ON` → FCL tests enabled

Only tests for enabled modules will be compiled and run.

## Adding New Tests

To add a new test case:

1. Open the appropriate test file
2. Add a new `TEST_F` or `TEST` block
3. Follow the existing pattern:
   ```cpp
   TEST_F(CollisionCheckerTest, YourNewTest) {
       auto checker = createCheckerFromYAML("path/to/config.yaml");
       auto trajectory = createTrajectory({{x, y, z}});
       EXPECT_TRUE/FALSE(checker->isCollision(trajectory));
   }
   ```
4. Rebuild and run tests

## Continuous Integration

These tests are designed to run in CI/CD pipelines. Example:

```yaml
# .github/workflows/test.yml
- name: Build and Test
  run: |
    mkdir build && cd build
    cmake .. -DBUILD_TESTING=ON
    cmake --build .
    ctest --output-on-failure
```

## Test Results

Tests output:
- ✅ Pass/Fail status for each test
- ⏱️ Execution time for performance tests
- 📊 Summary statistics

Example output:
```
[==========] Running 25 tests from 1 test suite.
[----------] Global test environment set-up.
[----------] 25 tests from OctreeCollisionCheckerTest
[ RUN      ] OctreeCollisionCheckerTest.ConstructorWithUAVConfig
[       OK ] OctreeCollisionCheckerTest.ConstructorWithUAVConfig (5 ms)
...
1000 collision checks took: 234 ms
[==========] 25 tests from 1 test suite ran. (1250 ms total)
[  PASSED  ] 25 tests.
```

## Troubleshooting

### Tests fail to find YAML files
- Ensure `TEST_DATA_DIR` is correctly set in CMakeLists.txt
- Check that test YAML files exist in `test/uav/` and `test/ugv/`

### Missing PNG files for Occupancy Map tests
- Some tests will be skipped if PNG files are missing
- This is expected behavior - tests use `GTEST_SKIP()` for missing resources

### Performance tests timeout
- Performance tests expect reasonable execution times
- Adjust thresholds in test code if running on slower hardware

## License

Same as parent project.
