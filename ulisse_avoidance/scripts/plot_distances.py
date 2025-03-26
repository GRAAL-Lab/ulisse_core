import sys
import os
import glob
import matplotlib.pyplot as plt
from rosbag_reader import read_positions_from_bag, read_path_data, get_bag_by_index, read_obstacles_from_bag
import numpy as np

import numpy as np
import matplotlib.pyplot as plt
import sys
import os
import glob
from rosbag_reader import read_positions_from_bag, read_obstacles_from_bag, get_bag_by_index

def rotate_point(px, py, cx, cy, yaw):
    """ Rotate point (px, py) around center (cx, cy) by yaw radians. """
    cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)
    dx, dy = px - cx, py - cy
    x_new = dx * cos_yaw + dy * sin_yaw + cx
    y_new = -dx * sin_yaw + dy * cos_yaw + cy
    return x_new, y_new

def closest_point_on_rectangle(asv_x, asv_y, obs_x, obs_y, yaw, size_x, size_y):
    """Compute the closest point on the obstacle rectangle to the ASV."""
    # Transform ASV into obstacle local frame
    local_asv_x, local_asv_y = rotate_point(asv_x, asv_y, obs_x, obs_y, -yaw)

    # Compute closest point within the rectangle in local frame
    half_x, half_y = size_x / 2, size_y / 2
    clamped_x = np.clip(local_asv_x, obs_x - half_x, obs_x + half_x)
    clamped_y = np.clip(local_asv_y, obs_y - half_y, obs_y + half_y)

    # Transform back to global frame
    global_clamped_x, global_clamped_y = rotate_point(clamped_x, clamped_y, obs_x, obs_y, yaw)

    #print(f"[DEBUG] ASV ({asv_x:.2f}, {asv_y:.2f}) -> Closest Point on Obstacle ({global_clamped_x:.2f}, {global_clamped_y:.2f})")

    return global_clamped_x, global_clamped_y

def compute_distances(positions_x, positions_y, timestamps, obstacles_data):
    """Compute distances between ASV and each obstacle at each timestamp."""
    distances = {}  # Dictionary {obs_id: [(timestamp, distance)]}

    #print("[DEBUG] Computing distances...")

    for obst_ts, obstacles in obstacles_data.items():
        # Find the closest ASV timestamp
        closest_asv_idx = min(range(len(timestamps)), key=lambda i: abs(timestamps[i] - obst_ts))
        closest_asv_x = positions_x[closest_asv_idx]
        closest_asv_y = positions_y[closest_asv_idx]
        closest_asv_ts = timestamps[closest_asv_idx]

        #print(f"\n[DEBUG] Obstacle at {obst_ts:.3f}s -> Closest ASV position at {closest_asv_ts:.3f}s")

        for obs_id, data in obstacles.items():
            obs_x, obs_y = data["position"]
            size_x, size_y = data["size"]
            yaw = data["yaw"]

            # Compute the closest point on the obstacle
            closest_point_x, closest_point_y = closest_point_on_rectangle(
                closest_asv_x, closest_asv_y, obs_x, obs_y, yaw, size_x, size_y
            )
            
            # Compute distance
            distance = np.sqrt((closest_asv_x - closest_point_x) ** 2 + (closest_asv_y - closest_point_y) ** 2)
            #print(f"  - ASV-Obstacle {obs_id} Distance: {distance:.2f}m")

            # ✅ Store distance for plotting
            if obs_id not in distances:
                distances[obs_id] = []
            distances[obs_id].append((obst_ts, distance))  # Store timestamp and distance

    return distances  # ✅ Now distances will be recorded!


def plot_distances(distances):
    """Plot ASV distance to each obstacle over time."""
    if not distances:
        print("[DEBUG] No distances recorded, skipping plot.")
        return

    plt.figure(figsize=(10, 5))
    
    for obs_id, dist_list in distances.items():
        ts_vals, dist_vals = zip(*dist_list)
        plt.plot(ts_vals, dist_vals, label=f"Obstacle {obs_id}")

    plt.xlabel("Time (s)")
    plt.ylabel("Distance (m)")
    plt.title("ASV Distance to Obstacles Over Time")
    plt.legend()
    plt.grid()
    plt.show()

def main():
    bag_directory = "/home/graal/ros2_ws/log/avoidance_bags"
    bag_index = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    bag_path = get_bag_by_index(bag_directory, bag_index)
    if not bag_path:
        print(f"Unable to find the bag file for index {bag_index}.")
        sys.exit(1)

    # Read ASV positions and obstacles
    #print("[DEBUG] Reading ASV positions and obstacle data...")
    positions_x, positions_y, timestamps = read_positions_from_bag(bag_path)
    obstacles_data = read_obstacles_from_bag(bag_path)

    #print(f"[DEBUG] Read {len(timestamps)} ASV positions and {len(obstacles_data)} obstacle timestamps.")

    # Compute distances and plot
    distances = compute_distances(positions_x, positions_y, timestamps, obstacles_data)
    plot_distances(distances)

if __name__ == "__main__":
    main()
