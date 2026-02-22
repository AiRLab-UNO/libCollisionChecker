#!/usr/bin/env python3
"""
Example usage of the collision_checker_py Python bindings with numpy arrays.
"""

import numpy as np
import collision_checker_py as cc

def example_quadtree_checker():
    """Example using QuadtreeCollisionChecker with numpy arrays."""
    print("=" * 60)
    print("Example: QuadtreeCollisionChecker with numpy arrays")
    print("=" * 60)
    
    # Create parameter manager from YAML config
    param_manager = cc.ParamManager("config.yaml")
    
    # Create a Quadtree collision checker (2D)
    checker = cc.QuadtreeCollisionChecker(param_manager)
    
    # Create a trajectory as a numpy array
    # Shape: (num_waypoints, dof) where dof = 2 for 2D
    trajectory = np.array([
        [0.0, 0.0],
        [1.0, 1.0],
        [2.0, 2.0],
        [3.0, 3.0],
        [4.0, 4.0]
    ])
    
    print(f"Trajectory shape: {trajectory.shape}")
    print(f"Trajectory:\n{trajectory}")
    
    # Check for collision
    is_collision = checker.is_collision(trajectory)
    print(f"\nCollision detected: {is_collision}")


def example_octree_checker():
    """Example using OctreeCollisionChecker with numpy arrays."""
    print("\n" + "=" * 60)
    print("Example: OctreeCollisionChecker with numpy arrays")
    print("=" * 60)
    
    # Create parameter manager from YAML config
    param_manager = cc.ParamManager("config.yaml")
    
    # Create an Octree collision checker (3D)
    checker = cc.OctreeCollisionChecker(param_manager)
    
    # Create a 3D trajectory as a numpy array
    # Shape: (num_waypoints, dof) where dof = 3 for 3D
    trajectory = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 1.0, 1.0],
        [2.0, 2.0, 2.0],
        [3.0, 3.0, 3.0],
    ])
    
    print(f"Trajectory shape: {trajectory.shape}")
    print(f"Trajectory:\n{trajectory}")
    
    # Check for collision
    is_collision = checker.is_collision(trajectory)
    print(f"\nCollision detected: {is_collision}")


def example_occupancy_map():
    """Example using OccupancyMap with numpy arrays."""
    print("\n" + "=" * 60)
    print("Example: OccupancyMap with numpy arrays")
    print("=" * 60)
    
    # Create parameter manager from YAML config
    param_manager = cc.ParamManager("config.yaml")
    
    # Create an occupancy map collision checker (2D)
    checker = cc.OccupancyMap(param_manager)
    
    # Create a 2D trajectory
    num_waypoints = 100
    t = np.linspace(0, 2*np.pi, num_waypoints)
    
    # Circular trajectory
    trajectory = np.column_stack([
        np.cos(t) * 5.0,  # x coordinates
        np.sin(t) * 5.0   # y coordinates
    ])
    
    print(f"Trajectory shape: {trajectory.shape}")
    print(f"First few waypoints:\n{trajectory[:5]}")
    
    # Check for collision
    is_collision = checker.is_collision(trajectory)
    print(f"\nCollision detected: {is_collision}")


def example_fcl_checker():
    """Example using FCL with numpy arrays."""
    print("\n" + "=" * 60)
    print("Example: FCL with numpy arrays")
    print("=" * 60)
    
    # Create parameter manager from YAML config
    param_manager = cc.ParamManager("config.yaml")
    
    # Create an FCL collision checker
    checker = cc.FCL(param_manager)
    
    # Create a trajectory with arbitrary DOF
    trajectory = np.array([
        [0.0, 0.0, 0.0, 0.0],
        [0.1, 0.1, 0.1, 0.1],
        [0.2, 0.2, 0.2, 0.2],
    ])
    
    print(f"Trajectory shape: {trajectory.shape}")
    print(f"Trajectory:\n{trajectory}")
    
    # Check for collision
    is_collision = checker.is_collision(trajectory)
    print(f"\nCollision detected: {is_collision}")


def example_vamp_checker():
    """Example using VampCollisionChecker with numpy arrays."""
    print("\n" + "=" * 60)
    print("Example: VampCollisionChecker with numpy arrays")
    print("=" * 60)
    
    # Create parameter manager from YAML config
    param_manager = cc.ParamManager("config.yaml")
    
    # Create a VAMP collision checker
    checker = cc.VampCollisionChecker(param_manager)
    
    # Create a trajectory (robot joint angles)
    trajectory = np.array([
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.5, 0.5, 0.5, 0.5, 0.5, 0.5],
        [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
    ])
    
    print(f"Trajectory shape: {trajectory.shape}")
    print(f"Trajectory:\n{trajectory}")
    
    # Check for collision
    is_collision = checker.is_collision(trajectory)
    print(f"\nCollision detected: {is_collision}")


def example_random_trajectory():
    """Example with randomly generated trajectory."""
    print("\n" + "=" * 60)
    print("Example: Random trajectory generation")
    print("=" * 60)
    
    # Create parameter manager from YAML config
    param_manager = cc.ParamManager("config.yaml")
    
    # Create a checker
    checker = cc.QuadtreeCollisionChecker(param_manager)
    
    # Generate random trajectory
    num_waypoints = 50
    dof = 2
    trajectory = np.random.randn(num_waypoints, dof) * 2.0
    
    print(f"Generated random trajectory with shape: {trajectory.shape}")
    print(f"Mean: {trajectory.mean(axis=0)}")
    print(f"Std: {trajectory.std(axis=0)}")
    
    # Check for collision
    is_collision = checker.is_collision(trajectory)
    print(f"\nCollision detected: {is_collision}")


if __name__ == "__main__":
    print("libCollisionChecker Python Bindings Examples")
    print("=" * 60)
    print(f"Module version: {cc.__version__}")
    print()
    
    # Note: These examples assume you have a valid config.yaml file
    # Comment out examples if you don't have the required configuration
    
    try:
        example_quadtree_checker()
    except Exception as e:
        print(f"QuadtreeCollisionChecker example failed: {e}")
    
    try:
        example_octree_checker()
    except Exception as e:
        print(f"OctreeCollisionChecker example failed: {e}")
    
    try:
        example_occupancy_map()
    except Exception as e:
        print(f"OccupancyMap example failed: {e}")
    
    try:
        example_fcl_checker()
    except Exception as e:
        print(f"FCL example failed: {e}")
    
    try:
        example_vamp_checker()
    except Exception as e:
        print(f"VampCollisionChecker example failed: {e}")
    
    try:
        example_random_trajectory()
    except Exception as e:
        print(f"Random trajectory example failed: {e}")
    
    print("\n" + "=" * 60)
    print("Examples complete!")
    print("=" * 60)
