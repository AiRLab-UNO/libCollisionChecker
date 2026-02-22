#!/usr/bin/env python3
"""
Interactive Matplotlib Collision Checker Demo

This example loads an environment from a YAML file and checks collision
against the mouse cursor position in real-time.

Usage:
    python interactive_collision_demo.py [yaml_file]
    
Example:
    python interactive_collision_demo.py test/ugv/env1.yaml
"""
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

dirpath = os.path.dirname(os.path.realpath(__file__))

sys.path.append(f"{dirpath}/../build")

import collision_checker_py as cc
import yaml
from pathlib import Path


class InteractiveCollisionChecker:
    def __init__(self, yaml_file):
        """Initialize the interactive collision checker with environment."""
        self.yaml_file = yaml_file
        
        # Load YAML to get environment info
        with open(yaml_file, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # Create parameter manager and collision checker
        self.param_manager = cc.ParamManager(yaml_file)

        # cc_type 

        
        # Determine which collision checker to use
        cc_type = self.config.get('collision_checker', 'quadtree').lower()
        
        # Map collision checker types
        if cc_type in ['quadtree', 'olc']:
            self.checker = cc.QuadtreeCollisionChecker(self.param_manager)
            print(f"Using QuadtreeCollisionChecker")
        elif cc_type == 'fcl':
            self.checker = cc.FCL(self.param_manager)
            print(f"Using FCL")
        elif cc_type == 'vamp':
            self.checker = cc.VampCollisionChecker(self.param_manager)
            print(f"Using VAMP")
        else:
            # Default to Quadtree
            self.checker = cc.QuadtreeCollisionChecker(self.param_manager)
            print(f"Using QuadtreeCollisionChecker (default)")
        
        # Extract environment info
        self.obstacles = np.array(self.config.get('obstacles', []))
        self.triangles = self.config.get('triangles', [])
        self.boundary = self.config.get('boundary', [-20, 1, -20, 2])
        self.robot_radius = self.config.get('robot_radius', 0.345)
        self.obstacle_size = self.config.get('obstacle_length', 0.25)
        self.start = self.config.get('start', [0, 0, 0])
        self.goal = self.config.get('goal', [0, 0])
        
        # Mouse position
        self.mouse_pos = None
        self.is_collision = False
        
        # Setup plot
        self.setup_plot()
        
    def setup_plot(self):
        """Setup the matplotlib figure and axes."""
        self.fig, self.ax = plt.subplots(figsize=(12, 10))
        self.fig.canvas.manager.set_window_title('Interactive Collision Checker - Move Mouse to Check Collision')
        
        # Set axis limits based on boundary
        x_min, x_max, y_min, y_max = self.boundary
        margin = 2.0
        self.ax.set_xlim(x_min - margin, x_max + margin)
        self.ax.set_ylim(y_min - margin, y_max + margin)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X (m)', fontsize=12)
        self.ax.set_ylabel('Y (m)', fontsize=12)
        
        # Draw environment
        self.draw_environment()
        
        # Create robot circle (will be updated with mouse position)
        self.robot_circle = patches.Circle(
            (0, 0), self.robot_radius, 
            fill=True, facecolor='green', alpha=0.5, 
            edgecolor='darkgreen', linewidth=2
        )
        self.ax.add_patch(self.robot_circle)
        
        # Create collision indicator
        self.collision_text = self.ax.text(
            0.02, 0.98, '', 
            transform=self.ax.transAxes,
            fontsize=14, fontweight='bold',
            verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8)
        )
        
        # Instructions text
        instructions = (
            "Move mouse cursor to check collision\n"
            "Green = No collision, Red = Collision\n"
            f"Robot radius: {self.robot_radius:.3f}m"
        )
        self.ax.text(
            0.02, 0.02, instructions,
            transform=self.ax.transAxes,
            fontsize=10,
            verticalalignment='bottom',
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8)
        )
        
        # Connect mouse events
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_mouse_move)
        
    def draw_environment(self):
        """Draw the environment: boundary, obstacles, triangles, start, goal."""
        # Draw boundary
        x_min, x_max, y_min, y_max = self.boundary
        boundary_rect = patches.Rectangle(
            (x_min, y_min), x_max - x_min, y_max - y_min,
            fill=False, edgecolor='black', linewidth=2, linestyle='--'
        )
        self.ax.add_patch(boundary_rect)
        
        # Draw obstacles (supports both point obstacles and rectangular obstacles)
        for obs in self.obstacles:
            if len(obs) >= 4:
                # Rectangular obstacle: [x, y, width, height]
                x, y, width, height = obs[0], obs[1], obs[2], obs[3]
                obs_rect = patches.Rectangle(
                    (x - width/2, y - height/2),
                    width, height,
                    fill=True, facecolor='gray', 
                    edgecolor='black', linewidth=1, alpha=0.7
                )
                self.ax.add_patch(obs_rect)
            elif len(obs) >= 2:
                # Point obstacle: [x, y]
                x, y = obs[0], obs[1]
                obs_rect = patches.Rectangle(
                    (x - self.obstacle_size, y - self.obstacle_size),
                    self.obstacle_size * 2, self.obstacle_size * 2,
                    fill=True, facecolor='gray', 
                    edgecolor='black', linewidth=1, alpha=0.7
                )
                self.ax.add_patch(obs_rect)
        
        # Draw triangles
        self.draw_triangles()
        
        # Draw start position
        if len(self.start) >= 2:
            self.ax.plot(
                self.start[0], self.start[1], 
                'go', markersize=15, label='Start',
                markeredgecolor='darkgreen', markeredgewidth=2
            )
        
        # Draw goal position
        if len(self.goal) >= 2:
            self.ax.plot(
                self.goal[0], self.goal[1], 
                'r*', markersize=20, label='Goal',
                markeredgecolor='darkred', markeredgewidth=1.5
            )
        
        self.ax.legend(loc='upper right', fontsize=10)
        self.ax.set_title(
            f"Environment: {self.config.get('name', 'Unknown')}\n"
            f"Collision Checker: {self.config.get('collision_checker', 'fcl')}",
            fontsize=14, fontweight='bold'
        )
    
    def draw_triangles(self):
        """Draw triangles from the environment configuration."""
        if not self.triangles:
            return
        
        for tri_idx, triangle in enumerate(self.triangles):
            if isinstance(triangle, list) and len(triangle) == 3:
                # triangle is a list of 3 vertices, each vertex is [x, y]
                vertices = np.array(triangle)
                polygon = patches.Polygon(
                    vertices,
                    fill=True, facecolor='lightblue',
                    edgecolor='blue', linewidth=1.5, alpha=0.6
                )
                self.ax.add_patch(polygon)
        
    def on_mouse_move(self, event):
        """Handle mouse move events."""
        if event.inaxes != self.ax:
            return
        
        # Update mouse position
        self.mouse_pos = np.array([event.xdata, event.ydata])
        
        # Check collision at this position
        self.check_collision()
        
        # Update visualization
        self.update_robot_visualization()
        
        # Redraw
        self.fig.canvas.draw_idle()
        
    def check_collision(self):
        """Check collision at current mouse position."""
        if self.mouse_pos is None:
            return
        
        # Validate mouse position
        if not np.isfinite(self.mouse_pos).all():
            return
        
        # Create trajectory - FCL needs 3D (x, y, z), others need 2D (x, y)
        if isinstance(self.checker, cc.FCL):
            # For FCL, add z=0 coordinate
            trajectory = np.array([[self.mouse_pos[0], self.mouse_pos[1], 0.0]])
        else:
            # For Quadtree and others, use 2D
            trajectory = self.mouse_pos.reshape(1, 2)
        
        # Check collision
        try:
            self.is_collision = self.checker.is_collision(trajectory)
        except Exception as e:
            print(f"Collision check error: {e}")
            print(f"Trajectory shape: {trajectory.shape}, values: {trajectory}")
            self.is_collision = False
            
    def update_robot_visualization(self):
        """Update robot circle and collision text."""
        if self.mouse_pos is None:
            return
        
        # Update robot position
        self.robot_circle.center = tuple(self.mouse_pos)
        
        # Update color based on collision
        if self.is_collision:
            self.robot_circle.set_facecolor('red')
            self.robot_circle.set_edgecolor('darkred')
            self.robot_circle.set_alpha(0.6)
            status_text = '⚠️ COLLISION'
            status_color = 'red'
        else:
            self.robot_circle.set_facecolor('green')
            self.robot_circle.set_edgecolor('darkgreen')
            self.robot_circle.set_alpha(0.5)
            status_text = '✓ SAFE'
            status_color = 'green'
        
        # Update collision text
        pos_text = f"Position: ({self.mouse_pos[0]:.2f}, {self.mouse_pos[1]:.2f})\n"
        self.collision_text.set_text(pos_text + status_text)
        self.collision_text.set_color(status_color)
        
    def run(self):
        """Start the interactive display."""
        plt.tight_layout()
        plt.show()


def main():
    """Main function."""
    # Default YAML file
    default_yaml = "/home/redwan/research/2026/ResearchWorkspace/BOT-Connect/Code/libcollisionchecker/test/ugv/env1.yaml"
    
    # Get YAML file from command line or use default
    if len(sys.argv) > 1:
        yaml_file = sys.argv[1]
    else:
        yaml_file = default_yaml
    
    # Check if file exists
    if not Path(yaml_file).exists():
        print(f"Error: YAML file not found: {yaml_file}")
        print(f"\nUsage: python {sys.argv[0]} [yaml_file]")
        print(f"Example: python {sys.argv[0]} test/ugv/env1.yaml")
        sys.exit(1)
    
    print("=" * 70)
    print("Interactive Collision Checker Demo")
    print("=" * 70)
    print(f"Loading environment: {yaml_file}")
    print("\nControls:")
    print("  - Move mouse cursor to check collision at that position")
    print("  - Green robot = Safe (no collision)")
    print("  - Red robot = Collision detected")
    print("\nClose the window to exit.")
    print("=" * 70)
    
    # Create and run the interactive checker
    try:
        app = InteractiveCollisionChecker(yaml_file)
        app.run()
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
