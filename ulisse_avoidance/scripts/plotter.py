import sys
import os
import glob
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.cm as cm
from rosbag_reader import (
    read_positions_from_bag,
    read_path_data,
    read_obstacles_from_bag,
    get_bag_by_index
)

# =======================
# Utility Functions
# =======================
def transform_to_world(bx, by, cx, cy, yaw):
    """Transform a point (bx, by) from the body frame to the world frame.
    (cx, cy) is the translation and yaw is the rotation angle."""
    cos_y, sin_y = np.cos(yaw), np.sin(yaw)
    wx = cos_y * bx - sin_y * by + cx
    wy = sin_y * bx + cos_y * by + cy
    return wx, wy

def transform_to_body(wx, wy, cx, cy, yaw):
    """Transform a point (wx, wy) from the world frame to a body frame centered at (cx, cy)
    with a rotation of yaw radians. Uses the fact that the inverse of a rotation matrix is its transpose."""
    dx, dy = wx - cx, wy - cy
    cos_y, sin_y = np.cos(yaw), np.sin(yaw)
    bx = cos_y * dx + sin_y * dy
    by = -sin_y * dx + cos_y * dy
    return bx, by

def closest_point_on_rectangle(asv_x, asv_y, obs_x, obs_y, yaw, size_x, size_y):
    """Compute the closest point on a rectangular obstacle to the ASV point.
    
    The obstacle is defined by its center (obs_x, obs_y), yaw, and dimensions size_x and size_y.
    """
    # Transform ASV position into obstacle's local (body) frame.
    local_x, local_y = transform_to_body(asv_x, asv_y, obs_x, obs_y, yaw)
    half_x, half_y = size_x / 2, size_y / 2
    clamped_x = np.clip(local_x, -half_x, half_x)
    clamped_y = np.clip(local_y, -half_y, half_y)
    # Transform the clamped point back to the global frame.
    global_x, global_y = transform_to_world(clamped_x, clamped_y, obs_x, obs_y, yaw)
    return global_x, global_y

# =======================
# Plotting Functions
# =======================
def plot_all(positions_x, positions_y, paths, start_time, closest_times=None, timestamps=None,
             obstacles_gt=None, obstacles_dt=None):
    fig = plt.figure(figsize=(5.5, 4))
    
    # Plot ASV trace
    if positions_x and positions_y:
        plt.plot(positions_x, positions_y, color='#ff7f0e', linestyle='-', linewidth=2, label="Trace")
    
    # Plot paths
    sorted_paths = sorted(paths.items(), key=lambda x: x[0])
    for idx, (timestamp, path_data) in enumerate(sorted_paths):
        if path_data:
            coords = path_data['coordinates']
            velocities = path_data.get('velocities', [])
            x_vals, y_vals = zip(*coords)
            #plt.plot(x_vals, y_vals, color='#1f77b4', linestyle='-', linewidth=2, label=f"Path {idx+1}")
            color_map = cm.get_cmap('Blues')
            color = color_map(idx / len(sorted_paths*0.
                                        ))  # Normalize index to get a gradient
            plt.plot(x_vals, y_vals, color=color, linestyle='-', linewidth=2, label=f"Path {idx+1}")

            # Plot velocity values at midpoints
            # if velocities and len(velocities) == len(coords) - 1:
            #     mid_x = [(x_vals[i] + x_vals[i+1]) / 2 for i in range(len(x_vals) - 1)]
            #     mid_y = [(y_vals[i] + y_vals[i+1]) / 2 for i in range(len(y_vals) - 1)]
            #     for mx, my, v in zip(mid_x, mid_y, velocities):
            #         plt.text(mx, my, f"{v:.2f}", fontsize=9, color='black', ha='center')
    
    # Plot ground truth obstacles as black lines with circular markers
    if obstacles_gt:
        for obs_id in {obs for obs_list in obstacles_gt.values() for obs in obs_list}:
            xs, ys = [], []
            for obst_ts, obs_dict in sorted(obstacles_gt.items()):
                if obs_id in obs_dict:
                    pos = obs_dict[obs_id]["position"]
                    xs.append(pos[0])
                    ys.append(pos[1])
            if xs and ys:
                plt.plot(xs, ys, linestyle='-', marker='o', color='black', linewidth=1,
                         markersize=2, label="GT Obstacles" if obs_id == 0 else "")
    
    # Plot detected obstacles as red lines with circular markers
    if obstacles_dt:
        for obs_id in {obs for obs_list in obstacles_dt.values() for obs in obs_list}:
            xs, ys = [], []
            for obst_ts, obs_dict in sorted(obstacles_dt.items()):
                if obs_id in obs_dict:
                    pos = obs_dict[obs_id]["position"]
                    xs.append(pos[0])
                    ys.append(pos[1])
            if xs and ys:
                plt.plot(xs, ys, linestyle='-', marker='o', color='#d62728', linewidth=1,
                         markersize=2, label="Detected Obstacles" if obs_id == 0 else "")
    
        # Mark ASV positions corresponding to the closest approach to obstacles
    if closest_times is not None and timestamps is not None:
        # To avoid duplicate legend entries, use a flag for the first marker.
        first_marker = True
        for ct in closest_times:
            # ct is a relative time; convert to absolute time for comparison.
            target_time = ct + start_time
            idx = min(range(len(timestamps)), key=lambda i: abs(timestamps[i] - target_time))
            marker_label = "Closest Approach" if first_marker else None
            plt.plot(positions_x[idx], positions_y[idx], marker='o', markersize=5,
                     color='#d62728', label=marker_label)
            first_marker = False
    
    x_min, x_max = plt.xlim()
    y_min, y_max = plt.ylim()
    pad = 10
    # plt.xlim(x_min - pad, x_max + pad)
    # plt.ylim(y_min - pad, y_max + pad)
    plt.xlim(-170, 20)
    plt.ylim(-25, 25)
    
    # dx = x_max - x_min
    # dy = y_max - y_min
    # if dx < dy:
    #     pad = (dy - dx) / 2
    #     plt.xlim(x_min - pad, x_max + pad)
    # elif dy < dx:
    #     pad = (dx - dy) / 2
    #     plt.ylim(y_min - pad, y_max + pad)
        
    plt.xlabel("East (m)")
    plt.ylabel("North (m)")
    #plt.title("ASV Path with Obstacles")
    plt.gca().set_facecolor('whitesmoke')
    plt.grid(True)
    #plt.legend(loc='upper right')
    plt.gca().set_aspect('equal', adjustable='box')
    return fig

def compute_closest_times(distances):
    """
    For each obstacle, return the relative time at which the minimum distance occurs.
    """
    closest_times = []
    for obs_id, dist_list in distances.items():
        ts, dists = zip(*dist_list)
        closest_times.append(min(zip(ts, dists), key=lambda x: x[1])[0])
    return closest_times

def compute_distances(positions_x, positions_y, timestamps, obstacles_data, start_time):
    """
    For each obstacle at each timestamp, compute the Euclidean distance from the ASV (using the nearest timestamp)
    to the closest point on the rectangular obstacle.
    """
    distances = {}  # {obs_id: [(relative_time, distance)]}
    
    for obst_ts, obs_dict in obstacles_data.items():
        # Find the ASV index that is closest in time to the obstacle timestamp.
        asv_idx = min(range(len(timestamps)), key=lambda i: abs(timestamps[i] - obst_ts))
        asv_x, asv_y = positions_x[asv_idx], positions_y[asv_idx]
        rel_ts = obst_ts - start_time
        
        for obs_id, data in obs_dict.items():
            obs_x, obs_y = data["position"]
            size_y, size_x = data["size"]
            yaw = data["yaw"]
            
            # Compute the closest point on the obstacle rectangle.
            cp_x, cp_y = closest_point_on_rectangle(asv_x, asv_y, obs_x, obs_y, yaw, size_x, size_y)
            
            # Compute the Euclidean distance.
            distance = np.sqrt((asv_x - cp_x) ** 2 + (asv_y - cp_y) ** 2)
            distances.setdefault(obs_id, []).append((rel_ts, distance))
    
    return distances

def plot_distances(distances_ground_truth, distances_detection, time_range=None):
    """Plot ASV distance to each obstacle over relative time for both ground truth and detection data."""
    fig = plt.figure(figsize=(5.5, 4))
    color_map = cm.get_cmap('tab10')
    
    # Plot distances for ground truth obstacles (segmented lines).
    for idx, (obs_id, dist_list) in enumerate(distances_ground_truth.items()):
        sorted_list = sorted(dist_list, key=lambda x: x[0])
        ts, dists = zip(*sorted_list)
        color = color_map(idx % 10)
        plt.plot(ts, dists, linestyle='--', color=color, label=f"GT-{obs_id}")
        min_idx = np.argmin(dists)
        plt.axvline(x=ts[min_idx], color=color, linestyle=':', label=f"GT Min {obs_id}")
    
    # Plot distances for detection obstacles (full lines).
    for idx, (obs_id, dist_list) in enumerate(distances_detection.items()):
        sorted_list = sorted(dist_list, key=lambda x: x[0])
        ts, dists = zip(*sorted_list)
        color = color_map((idx + len(distances_ground_truth)) % 10)
        plt.plot(ts, dists, linestyle='-', color=color, label=f"DTC-{obs_id}")
        min_idx = np.argmin(dists)
        plt.axvline(x=ts[min_idx], color=color, linestyle='-.', label=f"DTC Min {obs_id}")
    
    plt.axhline(y=10, color='#d62728', linestyle='--', linewidth=2)
    plt.xlabel("Time (s)")
    plt.ylabel("Distance (m)")

    #plt.title("ASV Distance to Obstacles Over Time")
    #plt.legend(loc='upper right')
    plt.gca().set_facecolor('whitesmoke')
    plt.grid(True)
    if time_range:
        plt.xlim(time_range)
        
    plt.xlim(30, 55)
    plt.ylim(5, 25)
    plt.yticks(np.arange(5, 26, 5))  # From 5 to 25 meters, with a step of 5 meters


    plt.gca().set_aspect('equal', adjustable='box')
    #plt.tight_layout()
    return fig

# =======================
# Main Function
# =======================
def main():
    bag_directory = "/home/graal/ros2_ws/log/avoidance_bags"
    bag_index = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    
    bag_path = get_bag_by_index(bag_directory, bag_index)
    if not bag_path:
        print(f"Unable to find the bag file for index {bag_index}.")
        sys.exit(1)
    
    # Read ASV positions and timestamps.
    positions_x, positions_y, timestamps = read_positions_from_bag(bag_path)
    if not timestamps:
        print("No timestamp data available.")
        sys.exit(1)
    
    # Determine start time: use the earliest path timestamp if available.
    paths = read_path_data(bag_path)
    start_time = min(paths.keys()) if paths else timestamps[0]
    
    # # Read obstacle data for ground truth and detection.
    obstacles_data_gt = read_obstacles_from_bag(bag_path, "/gt/obstacle_list")
    obstacles_data_dt = read_obstacles_from_bag(bag_path, "/trk/obstacle_list")
    
    # Compute distances.
    distances_gt = compute_distances(positions_x, positions_y, timestamps, obstacles_data_gt, start_time)
    distances_dt = compute_distances(positions_x, positions_y, timestamps, obstacles_data_dt, start_time)
    closest_times = compute_closest_times(distances_gt)
    
    time_min = 0.0
    time_max = max(timestamps) - start_time
    fig2 = plot_distances(distances_gt, distances_dt, time_range=(time_min, time_max))

    # closest_times = None
    # obstacles_data_gt = None
    # print("FUCKEEEEEEEEEEEEEEEEEEER")

    
    # Plot ASV path with obstacles and mark closest approaches.
    fig1 = plot_all(positions_x, positions_y, paths, start_time, closest_times, timestamps,
                    obstacles_gt=obstacles_data_gt, obstacles_dt=obstacles_data_dt)
    
    plt.show()

if __name__ == "__main__":
    main()
