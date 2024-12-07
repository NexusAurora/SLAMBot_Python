import numpy as np
import matplotlib.pyplot as plt

# Initialize occupancy grid (e.g., 100x100 grid with 0.5 as unknown)
occupancy_grid = np.full((100, 100), 0.5)

def update_grid(grid, pose, scan_data):
    """
    Update the occupancy grid with new scan data.
    grid: Occupancy grid (2D numpy array).
    pose: Current robot pose (x, y, theta).
    scan_data: List of (distance, angle) tuples.
    """
    x, y, theta = pose
    for distance, angle in scan_data:
        # Convert polar to cartesian
        scan_x = x + distance * np.cos(theta + angle)
        scan_y = y + distance * np.sin(theta + angle)
        
        # Update occupancy grid
        grid_x = int(scan_x) + grid.shape[0] // 2
        grid_y = int(scan_y) + grid.shape[1] // 2
        if 0 <= grid_x < grid.shape[0] and 0 <= grid_y < grid.shape[1]:
            grid[grid_x, grid_y] = 1  # Mark as occupied

    return grid

def detect_loop_closure(current_pose, past_poses, threshold=1.0):
    """
    Detect loop closure by comparing the current pose with past poses.
    current_pose: Current robot pose (x, y, theta).
    past_poses: List of previous poses.
    threshold: Distance threshold for loop closure detection.
    """
    for past_pose in past_poses:
        distance = np.linalg.norm(np.array(current_pose[:2]) - np.array(past_pose[:2]))
        if distance < threshold:
            return True, past_pose
    return False, None

# Example of robot movement and map update
pose = (0, 0, 0)  # Initial pose (x, y, theta)
scan_data = [(5, np.pi/4), (10, -np.pi/4)]  # Example scan data (distance, angle)
occupancy_grid = update_grid(occupancy_grid, pose, scan_data)

# Simulate movement
poses = [(0, 0, 0), (5, 0, np.pi/6), (10, 5, np.pi/4), (0, 0, 0)]  # Simulated path

for i, pose in enumerate(poses):
    occupancy_grid = update_grid(occupancy_grid, pose, scan_data)
    
    # Detect loop closure
    loop_detected, matching_pose = detect_loop_closure(pose, poses[:i])
    if loop_detected:
        print(f"Loop closure detected with pose {matching_pose}")

plt.imshow(occupancy_grid, cmap='gray')
plt.show()
