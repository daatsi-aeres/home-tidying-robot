#!/usr/bin/env python3
"""Generate occupancy grid map from home.sdf world geometry.
Includes walls and all grounded furniture visible at LiDAR height (~0.215m)."""
import numpy as np

RESOLUTION = 0.05
ORIGIN_X = -5.5
ORIGIN_Y = -3.5
WIDTH = 260
HEIGHT = 140

FREE = 254
OCCUPIED = 0
UNKNOWN = 205

def fill_rect(grid, x_min, y_min, x_max, y_max, value):
    c_lo = max(0, int(round((x_min - ORIGIN_X) / RESOLUTION)))
    c_hi = min(WIDTH, int(round((x_max - ORIGIN_X) / RESOLUTION)) + 1)
    r_hi = min(HEIGHT, HEIGHT - int(round((y_min - ORIGIN_Y) / RESOLUTION)))
    r_lo = max(0, HEIGHT - int(round((y_max - ORIGIN_Y) / RESOLUTION)) - 1)
    grid[r_lo:r_hi, c_lo:c_hi] = value

def fill_circle(grid, cx, cy, radius, value):
    for r in range(HEIGHT):
        for c in range(WIDTH):
            wx = ORIGIN_X + c * RESOLUTION
            wy = ORIGIN_Y + (HEIGHT - 1 - r) * RESOLUTION
            if (wx - cx)**2 + (wy - cy)**2 <= radius**2:
                grid[r, c] = value

def main():
    grid = np.full((HEIGHT, WIDTH), UNKNOWN, dtype=np.uint8)

    # --- Free space ---
    fill_rect(grid, -4.425, -2.425, 0.425, 2.425, FREE)   # Room 1
    fill_rect(grid, 0.575, -2.425, 6.425, 2.425, FREE)     # Room 2
    fill_rect(grid, 0.425, -0.75, 0.575, 0.75, FREE)       # Doorway

    # --- Walls ---
    walls = [
        (-4.5, 0.0,   0.15, 5.15),
        (-2.0, 2.5,   5.15, 0.15),
        (-2.0, -2.5,  5.15, 0.15),
        (0.5,  1.625, 0.15, 1.75),
        (0.5, -1.625, 0.15, 1.75),
        (6.5,  0.0,   0.15, 5.15),
        (3.5,  2.5,   6.15, 0.15),
        (3.5, -2.5,   6.15, 0.15),
    ]
    for cx, cy, sx, sy in walls:
        fill_rect(grid, cx-sx/2, cy-sy/2, cx+sx/2, cy+sy/2, OCCUPIED)

    # --- ROOM 1 furniture ---
    # Couch: (-3.8, 0), base+seat 0.6x1.6, back at (-4.05, 0) 0.12x1.6
    fill_rect(grid, -4.1, -0.8, -3.5, 0.8, OCCUPIED)
    fill_rect(grid, -4.11, -0.8, -3.99, 0.8, OCCUPIED)

    # Coffee table: (-2.5, 1.5), solid 0.8x0.5
    fill_rect(grid, -2.9, 1.25, -2.1, 1.75, OCCUPIED)

    # Side table: (-1.2, -1.8), solid 0.45x0.45
    fill_rect(grid, -1.425, -2.025, -0.975, -1.575, OCCUPIED)

    # Plant pot: (-0.2, 1.5), cylinder r=0.18
    fill_circle(grid, -0.2, 1.5, 0.18, OCCUPIED)

    # Collection box: (-3.5, -1.8), 0.50x0.40
    fill_rect(grid, -3.75, -2.0, -3.25, -1.6, OCCUPIED)

    # Shoe rack: (1.2, -0.2), 0.35x0.50 — just past doorway
    fill_rect(grid, 1.2-0.175, -0.2-0.25, 1.2+0.175, -0.2+0.25, OCCUPIED)

    # --- ROOM 2 furniture ---
    # Shelf: (3.5, 2.1), 0.8x0.30
    fill_rect(grid, 3.1, 1.95, 3.9, 2.25, OCCUPIED)

    # Desk: (5.5, 1.5), solid 1.2x0.6
    fill_rect(grid, 4.9, 1.2, 6.1, 1.8, OCCUPIED)

    # Armchair: (1.8, -1.5), rotated 0.3rad, seat 0.55x0.55 + back
    # Approximate as axis-aligned bounding box (slightly larger)
    fill_rect(grid, 1.8-0.35, -1.5-0.35, 1.8+0.35, -1.5+0.35, OCCUPIED)

    # Cabinet: (5.8, -1.0), 0.40x0.50
    fill_rect(grid, 5.6, -1.25, 6.0, -0.75, OCCUPIED)

    # Lamp stand: (4.5, 1.2), cylinder r=0.12
    fill_circle(grid, 4.5, 1.2, 0.12, OCCUPIED)

    # --- Save ---
    pgm_path = "src/tidybot/maps/home_map.pgm"
    with open(pgm_path, 'wb') as f:
        f.write(f"P5\n{WIDTH} {HEIGHT}\n255\n".encode())
        f.write(grid.tobytes())

    n_free = np.sum(grid == FREE)
    n_occ = np.sum(grid == OCCUPIED)
    n_unk = np.sum(grid == UNKNOWN)
    print(f"Saved {pgm_path} ({WIDTH}x{HEIGHT})")
    print(f"Cells: free={n_free} occ={n_occ} unk={n_unk}")

if __name__ == '__main__':
    main()
