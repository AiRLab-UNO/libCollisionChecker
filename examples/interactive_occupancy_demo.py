#!/usr/bin/env python3
"""
Interactive Occupancy Map Collision Checker Demo

This example loads an occupancy map (image-based) from a YAML/PNG file
and checks collision against the mouse cursor position in real-time.

Usage:
    python interactive_occupancy_demo.py [yaml_file]
    
Example:
    python interactive_occupancy_demo.py test/ugv/intel.yaml
    python interactive_occupancy_demo.py test/ugv/sr_clutter_01.yaml
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.image import imread
import sys
sys.path.append("../build")  # Adjust path to find the built Python bindings
import collision_checker_py as cc

import yaml
from pathlib import Path
import time


class InteractiveOccupancyChecker:
    def __init__(self, yaml_file):
        """Initialize the interactive occupancy map collision checker."""
        self.yaml_file = yaml_file
        self.yaml_dir = Path(yaml_file).parent
        
        # Load YAML to get environment info
        with open(yaml_file, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # Create parameter manager and collision checker
        self.param_manager = cc.ParamManager(yaml_file)
        self.checker = cc.OccupancyMap(self.param_manager)
        
        print(f"\nOccupancyMap Checker initialized")
        print(f"Robot radius: {self.config.get('robot_radius', 0.345)}")
        
        # Extract environment info
        self.robot_radius = self.config.get('robot_radius', 0.345)
        self.start = self.config.get('start', [0, 0, 0])
        self.goal = self.config.get('goal', [0, 0])
        self.resolution = self.config.get('resolution', 0.05)
        self.origin = self.config.get('origin', [0, 0])
        
        # Try to load the occupancy map image
        self.map_image = None
        self.load_map_image()
        
        # Mouse position
        self.mouse_pos = None
        self.is_collision = False
        self.last_check_time = 0
        self.check_interval = 0.05  # Check collision every 50ms max
        
        # Setup plot
        self.setup_plot()
        
    def load_map_image(self):
        """Load the occupancy map image if available."""
        # Check if YAML has explicit map_image_path
        if 'map_image_path' in self.config:
            image_path_from_yaml = self.config['map_image_path']
            yaml_path = Path(self.yaml_file).resolve()  # Get absolute path
            
            # If relative path, resolve relative to YAML file location
            if not Path(image_path_from_yaml).is_absolute():
                image_path = (yaml_path.parent / image_path_from_yaml).resolve()
            else:
                image_path = Path(image_path_from_yaml)
            
            if image_path.exists():
                try:
                    self.map_image = imread(str(image_path))
                    print(f"Loaded occupancy map image: {image_path}")
                    print(f"Image shape: {self.map_image.shape}")
                    # Extract resolution from config if available
                    if 'map_resolution' in self.config:
                        self.resolution = self.config['map_resolution']
                    return
                except Exception as e:
                    print(f"Warning: Could not load image {image_path}: {e}")
            else:
                print(f"Image file not found at: {image_path}")
        
        # Fallback: Look for image file with same stem as YAML
        yaml_path = Path(self.yaml_file).resolve()
        image_name = yaml_path.stem + '.png'
        image_path = yaml_path.parent / image_name
        
        if image_path.exists():
            try:
                self.map_image = imread(str(image_path))
                print(f"Loaded occupancy map image: {image_path}")
                print(f"Image shape: {self.map_image.shape}")
            except Exception as e:
                print(f"Warning: Could not load image {image_path}: {e}")
        else:
            print(f"No image file found at: {image_path}")
        
    def setup_plot(self):
        """Setup the matplotlib figure and axes."""
        self.fig, self.ax = plt.subplots(figsize=(14, 12))
        self.fig.canvas.manager.set_window_title('Interactive Occupancy Map - Move Mouse to Check Collision')
        
        # Display the map image if available
        if self.map_image is not None:
            # Calculate extent based on image size, resolution, and origin
            height, width = self.map_image.shape[:2]
            x_min = self.origin[0]
            y_min = self.origin[1]
            x_max = x_min + width * self.resolution
            y_max = y_min + height * self.resolution
            
            # Display image without flipping - keep same orientation as C++ library reads it
            self.ax.imshow(
                self.map_image, 
                extent=[x_min, x_max, y_min, y_max],
                cmap='gray', origin='upper', alpha=0.8
            )
            
            # Set axis limits
            self.ax.set_xlim(x_min, x_max)
            self.ax.set_ylim(y_min, y_max)
        else:
            # No image, use default limits
            self.ax.set_xlim(-10, 10)
            self.ax.set_ylim(-10, 10)
        
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X (m)', fontsize=12)
        self.ax.set_ylabel('Y (m)', fontsize=12)
        
        # Draw start and goal
        self.draw_markers()
        
        # Create robot circle (will be updated with mouse position)
        self.robot_circle = patches.Circle(
            (0, 0), self.robot_radius, 
            fill=True, facecolor='green', alpha=0.6, 
            edgecolor='darkgreen', linewidth=2, zorder=100
        )
        self.ax.add_patch(self.robot_circle)
        
        # Create collision indicator
        self.collision_text = self.ax.text(
            0.02, 0.98, '', 
            transform=self.ax.transAxes,
            fontsize=14, fontweight='bold',
            verticalalignment='top', zorder=200,
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.9, edgecolor='black', linewidth=2)
        )
        
        # Instructions text
        instructions = (
            "Move mouse cursor to check collision\n"
            "Green = No collision, Red = Collision\n"
            f"Robot radius: {self.robot_radius:.3f}m\n"
            f"Resolution: {self.resolution:.3f}m/pixel"
        )
        self.ax.text(
            0.02, 0.02, instructions,
            transform=self.ax.transAxes,
            fontsize=10,
            verticalalignment='bottom', zorder=200,
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9, edgecolor='black')
        )
        
        self.ax.set_title(
            f"Occupancy Map: {self.config.get('name', Path(self.yaml_file).stem)}\n"
            f"White = Free Space, Black = Obstacles",
            fontsize=14, fontweight='bold'
        )
        
        # Connect mouse events
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_mouse_move)
        
    def draw_markers(self):
        """Draw start and goal positions."""
        # Draw start position
        if len(self.start) >= 2:
            self.ax.plot(
                self.start[0], self.start[1], 
                'go', markersize=15, label='Start',
                markeredgecolor='darkgreen', markeredgewidth=2, zorder=150
            )
            # Draw start orientation if available
            if len(self.start) >= 3:
                arrow_len = self.robot_radius * 1.5
                dx = arrow_len * np.cos(self.start[2])
                dy = arrow_len * np.sin(self.start[2])
                self.ax.arrow(
                    self.start[0], self.start[1], dx, dy,
                    head_width=0.15, head_length=0.1, 
                    fc='darkgreen', ec='darkgreen', zorder=150
                )
        
        # Draw goal position
        if len(self.goal) >= 2:
            self.ax.plot(
                self.goal[0], self.goal[1], 
                'r*', markersize=20, label='Goal',
                markeredgecolor='darkred', markeredgewidth=1.5, zorder=150
            )
        
        if len(self.start) >= 2 or len(self.goal) >= 2:
            self.ax.legend(loc='upper right', fontsize=11, framealpha=0.9)
        
    def on_mouse_move(self, event):
        """Handle mouse move events."""
        if event.inaxes != self.ax:
            return
        
        # Update mouse position
        self.mouse_pos = np.array([event.xdata, event.ydata])
        
        # Throttle collision checks to improve performance
        current_time = time.time()
        if current_time - self.last_check_time >= self.check_interval:
            # Check collision at this position
            self.check_collision()
            self.last_check_time = current_time
        
        # Always update visualization (but collision status only changes when checked)
        self.update_robot_visualization()
        
        # Use blit for faster rendering (only update changed artists)
        self.fig.canvas.draw_idle()
    
    def check_collision(self):
        """Check collision at current mouse position."""
        if self.mouse_pos is None:
            return
        
        # Validate mouse position
        if not np.isfinite(self.mouse_pos).all():
            return
        
        # Create a simple trajectory (single point)
        trajectory = self.mouse_pos.reshape(1, 2)
        
        # Check collision
        try:
            # The collision checker should work with world coordinates
            # It handles the conversion to image/map coordinates internally
            self.is_collision = self.checker.is_collision(trajectory)
            
        except Exception as e:
            print(f"Collision check error: {e}")
            print(f"Trajectory shape: {trajectory.shape}, values: {trajectory}")
            self.is_collision = False
    
    def world_to_map_debug(self, x, y):
        """Convert world coords to map coords (for debugging).
        
        Matches world_to_image transform used in trajectory visualization.
        Uses origin='upper' with y-flip for proper image indexing.
        """
        map_x = int((x - self.origin[0]) / self.resolution)
        map_y = int((y - self.origin[1]) / self.resolution)
        
        # When using origin='upper' in imshow, flip the y coordinate for image indexing
        if self.map_image is not None:
            img_height = self.map_image.shape[0]
            map_y = img_height - map_y
        
        return map_x, map_y
            
    def get_pixel_from_world(self, world_pos):
        """Convert world coordinates to image pixel coordinates."""
        if self.map_image is None:
            return None
        
        # World position relative to origin
        rel_x = world_pos[0] - self.origin[0]
        rel_y = world_pos[1] - self.origin[1]
        
        # Convert to pixel coordinates
        pixel_x = int(rel_x / self.resolution)
        pixel_y = int(rel_y / self.resolution)
        
        # Image is stored as [height, width], where height corresponds to y
        height, width = self.map_image.shape[:2]
        
        # Flip y coordinate (image origin is top-left, world origin is bottom-left)
        pixel_y = height - pixel_y
        
        return pixel_x, pixel_y
                
    def update_robot_visualization(self):
        """Update robot circle and collision text."""
        if self.mouse_pos is None:
            return
        
        # Update robot position
        self.robot_circle.center = tuple(self.mouse_pos)
        
        # Check if position is within map bounds
        within_bounds = True
        pixel_info = None
        
        if self.map_image is not None:
            map_x, map_y = self.world_to_map_debug(self.mouse_pos[0], self.mouse_pos[1])
            height, width = self.map_image.shape[:2]
            within_bounds = 0 <= map_x < width and 0 <= map_y < height
            
            # Get pixel value if within bounds
            if within_bounds:
                pixel_val = self.map_image[map_y, map_x]
                if isinstance(pixel_val, np.ndarray):
                    # Handle both normalized (0-1) and uint8 (0-255) pixel values
                    if pixel_val.dtype == np.float32 or pixel_val.dtype == np.float64:
                        # Normalized float values
                        r = int(pixel_val[0] * 255)
                        g = int(pixel_val[1] * 255)
                        b = int(pixel_val[2] * 255)
                    else:
                        # Uint8 values
                        r = int(pixel_val[0])
                        g = int(pixel_val[1])
                        b = int(pixel_val[2])
                else:
                    # Grayscale
                    if isinstance(pixel_val, (float, np.floating)):
                        r = g = b = int(pixel_val * 255)
                    else:
                        r = g = b = int(pixel_val)
                
                # Threshold comparison (200 is from color_threshold in YAML)
                brightness = (r + g + b) // 3
                is_dark = (r < 200) and (g < 200) and (b < 200)
                pixel_info = (r, g, b, brightness, is_dark, map_x, map_y, width, height)
        
        # Update color based on collision
        if self.is_collision:
            self.robot_circle.set_facecolor('red')
            self.robot_circle.set_edgecolor('darkred')
            self.robot_circle.set_alpha(0.7)
            if within_bounds:
                status_text = '⚠️ COLLISION (Obstacle)'
            else:
                status_text = '⚠️ OUT OF BOUNDS'
            status_color = 'red'
        else:
            self.robot_circle.set_facecolor('green')
            self.robot_circle.set_edgecolor('darkgreen')
            self.robot_circle.set_alpha(0.6)
            status_text = '✓ SAFE (Free Space)'
            status_color = 'green'
        
        # Update collision text with detailed debug info
        pos_text = f"World: ({self.mouse_pos[0]:.3f}, {self.mouse_pos[1]:.3f})\n"
        
        # Add map coordinates and pixel information
        if pixel_info:
            r, g, b, brightness, is_dark, map_x, map_y, width, height = pixel_info
            pos_text += f"Map: ({map_x}, {map_y}) / Size: {width}×{height}\n"
            pos_text += f"Pixel: RGB({r},{g},{b}) [{brightness}]\n"
            pos_text += f"{'Obstacle (dark)' if is_dark else 'Free (bright)'}"
        elif self.map_image is not None:
            pos_text += f"[Out of bounds]"
        
        self.collision_text.set_text(pos_text + "\n" + status_text)
        self.collision_text.set_color(status_color)
        
    def run(self):
        """Start the interactive display."""
        plt.tight_layout()
        plt.show()


def main():
    """Main function."""
    # Default YAML file
    default_yaml = "/home/redwan/research/2026/ResearchWorkspace/BOT-Connect/Code/libcollisionchecker/test/ugv/intel.yaml"
    
    # Get YAML file from command line or use default
    if len(sys.argv) > 1:
        yaml_file = sys.argv[1]
    else:
        yaml_file = default_yaml
    
    # Check if file exists
    if not Path(yaml_file).exists():
        print(f"Error: YAML file not found: {yaml_file}")
        print(f"\nUsage: python {sys.argv[0]} [yaml_file]")
        print(f"Example: python {sys.argv[0]} test/ugv/intel.yaml")
        print(f"Example: python {sys.argv[0]} test/ugv/sr_clutter_01.yaml")
        sys.exit(1)
    
    print("=" * 70)
    print("Interactive Occupancy Map Collision Checker Demo")
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
        app = InteractiveOccupancyChecker(yaml_file)
        app.run()
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
