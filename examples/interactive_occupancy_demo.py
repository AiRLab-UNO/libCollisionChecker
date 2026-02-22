#!/usr/bin/env python3
"""
Interactive Occupancy Map Collision Checker

Usage:
    python interactive_occupancy_demo.py [yaml_file]
"""

import sys
import time
import yaml
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.image import imread
from pathlib import Path

sys.path.append("../build")
import collision_checker_py as cc


class InteractiveOccupancyChecker:
    def __init__(self, yaml_file):
        with open(yaml_file) as f:
            cfg = yaml.safe_load(f)

        self.cfg        = cfg
        self.resolution = cfg.get("map_resolution", 0.05)
        self.origin     = cfg.get("origin", [0, 0])
        self.robot_r    = cfg.get("robot_radius", 0.345)
        self.start      = cfg.get("start", [0, 0, 0])
        self.goal       = cfg.get("goal",  [0, 0])

        self.checker = cc.OccupancyMap(cc.ParamManager(yaml_file))

        # Load map image (same-stem PNG fallback)
        img_path = Path(yaml_file).with_suffix(".png")
        rel = cfg.get("map_image_path")
        if rel:
            img_path = (Path(yaml_file).parent / rel).resolve()
        self.map_img = imread(str(img_path)) if img_path.exists() else None

        self._last_check = 0
        self._interval   = 0.05          # collision check throttle (s)
        self._collision  = False
        self._setup_plot()

    # ------------------------------------------------------------------ coords
    def _img_to_world(self, px, py):
        """Image pixel → world (x, y).  No y-flip (origin='upper' but we don't flip)."""
        return px * self.resolution + self.origin[0], \
               py * self.resolution + self.origin[1]

    # ------------------------------------------------------------------ plot
    def _setup_plot(self):
        self.fig, self.ax = plt.subplots(figsize=(12, 10))
        self.fig.canvas.manager.set_window_title("Occupancy Map – Move Mouse")

        if self.map_img is not None:
            self.ax.imshow(self.map_img, cmap="gray", origin="upper", alpha=0.85)

        # Start / goal markers
        if len(self.start) >= 2:
            self.ax.plot(*self.start[:2], "go", ms=12, zorder=10,
                         markeredgecolor="darkgreen", markeredgewidth=2, label="Start")
        if len(self.goal) >= 2:
            self.ax.plot(*self.goal[:2], "r*", ms=18, zorder=10,
                         markeredgecolor="darkred", markeredgewidth=1.5, label="Goal")
        if len(self.start) >= 2 or len(self.goal) >= 2:
            self.ax.legend(loc="upper right", fontsize=10, framealpha=0.9)

        # Robot circle (radius in pixels for image-space display)
        r_px = self.robot_r / self.resolution
        self.robot_patch = patches.Circle((0, 0), r_px,
                                          facecolor="green", edgecolor="darkgreen",
                                          alpha=0.65, linewidth=2, zorder=50)
        self.ax.add_patch(self.robot_patch)

        # Status text (top-left)
        self.status_txt = self.ax.text(
            0.02, 0.98, "", transform=self.ax.transAxes,
            fontsize=13, fontweight="bold", va="top", zorder=100,
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.85, edgecolor="black")
        )

        self.ax.set_title(
            f"Map: {self.cfg.get('name', Path(self.cfg.get('image','')).stem)}  |  "
            f"Robot r={self.robot_r:.3f} m  |  Res={self.resolution:.3f} m/px",
            fontsize=12
        )
        self.fig.canvas.mpl_connect("motion_notify_event", self._on_mouse_move)

    # ------------------------------------------------------------------ events
    def _on_mouse_move(self, event):
        if event.inaxes is not self.ax or event.xdata is None:
            return

        px, py = event.xdata, event.ydata

        # Move robot circle (image-space)
        self.robot_patch.center = (px, py)

        # Convert to world coords for collision check
        wx, wy = self._img_to_world(px, py)

        now = time.monotonic()
        if now - self._last_check >= self._interval:
            try:
                self._collision = self.checker.is_collision(
                    np.array([[wx, wy]], dtype=float)
                )
            except Exception:
                self._collision = False
            self._last_check = now

        # Update visuals
        if self._collision:
            self.robot_patch.set_facecolor("red")
            self.robot_patch.set_edgecolor("darkred")
            self.status_txt.set_text("⚠ COLLISION")
            self.status_txt.set_color("red")
        else:
            self.robot_patch.set_facecolor("green")
            self.robot_patch.set_edgecolor("darkgreen")
            self.status_txt.set_text(f"✓ SAFE  ({wx:.2f}, {wy:.2f})")
            self.status_txt.set_color("darkgreen")

        self.fig.canvas.draw_idle()

    def run(self):
        plt.tight_layout()
        plt.show()


def main():
    default = "/home/redwan/research/2026/ResearchWorkspace/BOT-Connect/Code/libcollisionchecker/test/ugv/sr_clutter_01.yaml"
    yaml_file = sys.argv[1] if len(sys.argv) > 1 else default

    if not Path(yaml_file).exists():
        print(f"File not found: {yaml_file}")
        sys.exit(1)

    InteractiveOccupancyChecker(yaml_file).run()


if __name__ == "__main__":
    main()