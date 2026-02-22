#!/usr/bin/env python3
"""
Interactive Trajectory Collision Checker Demo

Click to add waypoints and check if the trajectory is collision-free.

Usage:
    python interactive_trajectory_demo.py [yaml_file]
    
Example:
    python interactive_trajectory_demo.py test/ugv/env1.yaml
"""
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.widgets import Button
sys.path.append("../build")
import collision_checker_py as cc

import yaml
from pathlib import Path


class InteractiveTrajectoryChecker:
    def __init__(self, yaml_file):
        """Initialize the interactive trajectory collision checker."""
        self.yaml_file = yaml_file
        
        # Load YAML to get environment info
        with open(yaml_file, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # Create parameter manager and collision checker
        self.param_manager = cc.ParamManager(yaml_file)
        self.checker = cc.QuadtreeCollisionChecker(self.param_manager)
        
        # Extract environment info
        self.obstacles = np.array(self.config.get('obstacles', []))
        self.boundary = self.config.get('boundary', [-20, 1, -20, 2])
        self.robot_radius = self.config.get('robot_radius', 0.345)
        self.obstacle_size = self.config.get('obstacle_length', 0.25)
        self.start = self.config.get('start', [0, 0, 0])
        self.goal = self.config.get('goal', [0, 0])
        
        # Trajectory waypoints
        self.waypoints = []
        self.is_collision = False
        
        # Setup plot
        self.setup_plot()
        
    def setup_plot(self):
        """Setup the matplotlib figure and axes."""
        self.fig, self.ax = plt.subplots(figsize=(12, 10))
        self.fig.canvas.manager.set_window_title('Interactive Trajectory Checker - Click to Add Waypoints')
        
        # Set axis limits
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
        
        # Trajectory line
        self.trajectory_line, = self.ax.plot([], [], 'b-', linewidth=2, label='Trajectory')
        self.waypoint_plot, = self.ax.plot([], [], 'bo', markersize=10, label='Waypoints')
        
        # Create collision indicator
        self.collision_text = self.ax.text(
            0.02, 0.98, 'Click to add waypoints', 
            transform=self.ax.transAxes,
            fontsize=14, fontweight='bold',
            verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8)
        )
        
        # Instructions text
        instructions = (
            "Left Click: Add waypoint\n"
            "Right Click: Remove last waypoint\n"
            "Green = Safe, Red = Collision\n"
            f"Robot radius: {self.robot_radius:.3f}m"
        )
        self.ax.text(
            0.02, 0.02, instructions,
            transform=self.ax.transAxes,
            fontsize=10,
            verticalalignment='bottom',
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8)
        )
        
        # Add buttons
        ax_clear = plt.axes([0.7, 0.02, 0.12, 0.04])
        ax_check = plt.axes([0.83, 0.02, 0.15, 0.04])
        self.btn_clear = Button(ax_clear, 'Clear')
        self.btn_check = Button(ax_check, 'Check Collision')
        self.btn_clear.on_clicked(self.clear_trajectory)
        self.btn_check.on_clicked(self.check_collision_button)
        
        self.ax.legend(loc='upper right', fontsize=10)
        
        # Connect mouse events
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        
    def draw_environment(self):
        """Draw the environment: boundary, obstacles, start, goal."""
        # Draw boundary
        x_min, x_max, y_min, y_max = self.boundary
        boundary_rect = patches.Rectangle(
            (x_min, y_min), x_max - x_min, y_max - y_min,
            fill=False, edgecolor='black', linewidth=2, linestyle='--'
        )
        self.ax.add_patch(boundary_rect)
        
        # Draw obstacles
        for obs in self.obstacles:
            if len(obs) >= 2:
                x, y = obs[0], obs[1]
                obs_rect = patches.Rectangle(
                    (x - self.obstacle_size/2, y - self.obstacle_size/2),
                    self.obstacle_size, self.obstacle_size,
                    fill=True, facecolor='gray', 
                    edgecolor='black', linewidth=1, alpha=0.7
                )
                self.ax.add_patch(obs_rect)
        
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
        
        self.ax.set_title(
            f"Environment: {self.config.get('name', 'Unknown')} - Click to Create Trajectory",
            fontsize=14, fontweight='bold'
        )
        
    def on_click(self, event):
        """Handle mouse click events."""
        if event.inaxes != self.ax:
            return
        
        if event.button == 1:  # Left click - add waypoint
            self.waypoints.append([event.xdata, event.ydata])
            self.update_trajectory_plot()
            self.check_collision()
        elif event.button == 3:  # Right click - remove last waypoint
            if self.waypoints:
                self.waypoints.pop()
                self.update_trajectory_plot()
                self.check_collision()
        
    def update_trajectory_plot(self):
        """Update trajectory visualization."""
        if not self.waypoints:
            self.trajectory_line.set_data([], [])
            self.waypoint_plot.set_data([], [])
        else:
            waypoints_array = np.array(self.waypoints)
            self.trajectory_line.set_data(waypoints_array[:, 0], waypoints_array[:, 1])
            self.waypoint_plot.set_data(waypoints_array[:, 0], waypoints_array[:, 1])
        
        self.fig.canvas.draw_idle()
        
    def check_collision(self):
        """Check collision for the current trajectory."""
        if len(self.waypoints) < 1:
            self.collision_text.set_text('Click to add waypoints')
            self.collision_text.set_color('black')
            self.trajectory_line.set_color('blue')
            self.waypoint_plot.set_color('blue')
            return
        
        # Create trajectory as numpy array
        trajectory = np.array(self.waypoints)
        
        # Check collision
        try:
            self.is_collision = self.checker.is_collision(trajectory)
        except Exception as e:
            print(f"Collision check error: {e}")
            self.is_collision = False
        
        # Update visualization
        if self.is_collision:
            status_text = f'⚠️ COLLISION ({len(self.waypoints)} waypoints)'
            status_color = 'red'
            line_color = 'red'
        else:
            status_text = f'✓ SAFE ({len(self.waypoints)} waypoints)'
            status_color = 'green'
            line_color = 'green'
        
        self.collision_text.set_text(status_text)
        self.collision_text.set_color(status_color)
        self.trajectory_line.set_color(line_color)
        self.waypoint_plot.set_color(line_color)
        
        self.fig.canvas.draw_idle()
        
    def check_collision_button(self, event):
        """Handle check collision button press."""
        self.check_collision()
        
    def clear_trajectory(self, event):
        """Clear all waypoints."""
        self.waypoints = []
        self.update_trajectory_plot()
        self.collision_text.set_text('Click to add waypoints')
        self.collision_text.set_color('black')
        self.trajectory_line.set_color('blue')
        self.waypoint_plot.set_color('blue')
        self.fig.canvas.draw_idle()
        
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
    print("Interactive Trajectory Collision Checker Demo")
    print("=" * 70)
    print(f"Loading environment: {yaml_file}")
    print("\nControls:")
    print("  - Left Click: Add waypoint to trajectory")
    print("  - Right Click: Remove last waypoint")
    print("  - Click 'Clear' button to reset")
    print("  - Click 'Check Collision' to verify")
    print("  - Green trajectory = Safe (no collision)")
    print("  - Red trajectory = Collision detected")
    print("\nClose the window to exit.")
    print("=" * 70)
    
    # Create and run the interactive checker
    try:
        app = InteractiveTrajectoryChecker(yaml_file)
        app.run()
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
