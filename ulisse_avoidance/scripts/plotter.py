import sys
import os
import glob
import numpy as np
import matplotlib.pyplot as plt
from rosbag_reader import read_positions_from_bag, read_path_data, read_obstacles_from_bag, get_bag_by_index
import matplotlib.cm as cm

# =======================
# Utility Functions
# =======================
def DistObstASVInWorldFrame(bF_px, bF_py, wF_cx, wF_cy, yaw):
    """Transform point from body frame to world frame."""
    wTb = np.array([
        [np.cos(yaw), -np.sin(yaw), wF_cx],
        [np.sin(yaw),  np.cos(yaw), wF_cy],
        [0, 0, 1]
    ])
    
    # Correcting the transformation
    w_P = np.matmul(wTb, np.array([[bF_px], [bF_py], [1]]))
    
    return w_P[0, 0], w_P[1, 0]

def DistObstASVInBodyFrame(wF_px, wF_py, wF_cx, wF_cy, yaw):
    """Rotate point (px, py) around center (cx, cy) by yaw radians."""
    wTb = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    # Compute the displacement
    wF_dx, wF_dy = wF_px - wF_cx, wF_py - wF_cy
    
    # Transform the point
    b_P = np.matmul(np.linalg.inv(wTb), np.array([[wF_dx], [wF_dy], [1]]))
    
    return b_P[0, 0], b_P[1, 0]  # Extract x, y components

def closest_point_on_rectangle(asv_x, asv_y, obs_x, obs_y, yaw, size_x, size_y):
    """Compute the closest point on the obstacle rectangle to the ASV."""
    
    #return obs_x, obs_y
    # Transform ASV into the obstacle's local frame
    local_asv_x, local_asv_y = DistObstASVInBodyFrame(asv_x, asv_y, obs_x, obs_y, yaw)
    
    # Compute closest point within the rectangle in the local frame
    half_x, half_y = size_x / 2, size_y / 2
    clamped_x = np.clip(local_asv_x, -half_x, half_x)
    clamped_y = np.clip(local_asv_y, -half_y, half_y)

    
    # Transform back to global frame
    global_clamped_x, global_clamped_y = DistObstASVInWorldFrame(clamped_x, clamped_y, obs_x, obs_y, yaw)
    return global_clamped_x, global_clamped_y

# =======================
# Plotting Functions
# =======================
def plot_all(positions_x, positions_y, paths, start_time, closest_times=None, timestamps=None):
    """
    Plot ASV positions and multiple path instances on a spatial (XY) plot.
    Points where the ASV is closest to an obstacle are colored differently.
    """
    fig = plt.figure(figsize=(5.5, 4)) 
    
    # Plot ASV trace (orange)
    if positions_x and positions_y:
        plt.plot(positions_x, positions_y, color='#ff7f0e', linestyle='-', linewidth=2, label="Trace")
        
        # If both closest_times and timestamps are provided, compute relative times and highlight points
        if closest_times and timestamps:
            relative_times = np.array(timestamps) - start_time
            tol = 1.0  # tolerance of 1 second
            for ct in closest_times:
                # Find the index with the minimum difference to the current closest time
                diffs = np.abs(relative_times - ct)
                idx = np.argmin(diffs)
                if diffs[idx] < tol:
                    plt.scatter(positions_x[idx], positions_y[idx], color='#d62728', s=50, zorder=5)
                    # Optionally, print for debugging:
                    # print(f"Highlighting point at index {idx} with relative time {relative_times[idx]:.2f}s (diff {diffs[idx]:.2f}s)")
        else:
            # Fallback: highlight the first point
            plt.scatter(positions_x[0], positions_y[0], c='#ff7f0e', marker='o', s=50, edgecolors='black', zorder=5)
    
    # Plot paths in blue
    sorted_paths = sorted(paths.items(), key=lambda x: x[0])
    for idx, (timestamp, path_data) in enumerate(sorted_paths):
        if path_data:
            coordinates = path_data['coordinates']
            velocities = path_data['velocities']
            x_vals, y_vals = zip(*coordinates)
            delay = timestamp - start_time  # now relative to the first path timestamp
            label = f"Path {idx+1} (t={delay:.1f}s)"
            plt.plot(x_vals, y_vals, color='#1f77b4', linestyle='-', linewidth=2, label=label)

            # Annotate velocity at midpoints for each segment
            for i in range(len(coordinates) - 1):
                x1, y1 = coordinates[i]
                x2, y2 = coordinates[i + 1]
                midpoint_x = (x1 + x2) / 2
                midpoint_y = (y1 + y2) / 2
                velocity = velocities[i] if i < len(velocities) else 0.0
                plt.text(midpoint_x, midpoint_y, f'{velocity:.2f} m/s', color='black', fontsize=9,
                         ha='center', va='bottom')
    
    # Labels, grid and title
    plt.xlabel("East (m)")
    plt.ylabel("North (m)")
    plt.title("ASV Path")
    plt.grid(True, which='both')
    plt.gca().set_aspect('equal', adjustable='box')
    plt.gca().set_facecolor('whitesmoke')
    
    # Adjust axes limits based on all points from trace and paths
    all_x_vals = positions_x.copy()
    all_y_vals = positions_y.copy()
    for _, path_data in sorted_paths:
        if path_data:
            coordinates = path_data['coordinates']
            x_vals, y_vals = zip(*coordinates)
            all_x_vals.extend(x_vals)
            all_y_vals.extend(y_vals)
    
    data_x_min, data_x_max = min(all_x_vals), max(all_x_vals)
    data_y_min, data_y_max = min(all_y_vals), max(all_y_vals)
    width_x = data_x_max - data_x_min
    width_y = data_y_max - data_y_min
    margin_x = width_x * 0.1 if width_x > 0 else 1
    margin_y = width_y * 0.1 if width_y > 0 else 1
    x_min = data_x_min - margin_x
    x_max = data_x_max + margin_x
    y_min = data_y_min - margin_y
    y_max = data_y_max + margin_y

    # Ensure minimum span of 50 meters
    if (x_max - x_min) < 50:
        extra = (50 - (x_max - x_min)) / 2
        x_min -= extra
        x_max += extra
    if (y_max - y_min) < 50:
        extra = (50 - (y_max - y_min)) / 2
        y_min -= extra
        y_max += extra

    plt.xlim(x_min, x_max)
    plt.ylim(y_min, y_max)
    
    # Set ticks as multiples of 20
    tick_start_x = np.floor(x_min / 20) * 20
    tick_end_x = np.ceil(x_max / 20) * 20
    tick_start_y = np.floor(y_min / 20) * 20
    tick_end_y = np.ceil(y_max / 20) * 20
    plt.xticks(np.arange(tick_start_x, tick_end_x + 1, 20))
    plt.yticks(np.arange(tick_start_y, tick_end_y + 1, 20))
    
    plt.tight_layout()
    plt.legend(loc='upper right')
    
    return fig


def compute_closest_times(distances):
    """
    For each obstacle, return the relative time at which the minimum distance occurs.
    Returns a list of timestamps.
    """
    closest_times = []
    for obs_id, dist_list in distances.items():
        ts_vals, dist_vals = zip(*dist_list)
        min_idx = np.argmin(dist_vals)
        closest_times.append(ts_vals[min_idx])
    return closest_times

def compute_distances(positions_x, positions_y, timestamps, obstacles_data, start_time):
    distances = {}  # Dictionary {obs_id: [(relative_time, distance)]}
    
    for obst_ts, obstacles in obstacles_data.items():
        # Only include obstacle timestamps that are at or after the start_time
        # if obst_ts < start_time:
        #     continue
        
        # # Find the ASV position closest in time to the obstacle timestamp
        # from scipy.interpolate import interp1d

        # # Create interpolation functions
        # interp_x = interp1d(timestamps, positions_x, kind='linear', fill_value="extrapolate")
        # interp_y = interp1d(timestamps, positions_y, kind='linear', fill_value="extrapolate")

        # # Interpolate ASV position at obstacle timestamp
        # closest_asv_x = interp_x(obst_ts)
        # closest_asv_y = interp_y(obst_ts)

        closest_asv_idx = min(range(len(timestamps)), key=lambda i: abs(timestamps[i] - obst_ts))
        closest_asv_x = positions_x[closest_asv_idx]
        closest_asv_y = positions_y[closest_asv_idx]
        relative_ts = obst_ts - start_time
        
        for obs_id, data in obstacles.items():
            obs_x, obs_y = data["position"]
            size_x, size_y = data["size"]
            yaw = data["yaw"]
            
            # Compute the closest point on the rectangle obstacle
            closest_point_x, closest_point_y = closest_point_on_rectangle(
                closest_asv_x, closest_asv_y, obs_x, obs_y, yaw, size_x, size_y
            )
            
            # Compute the Euclidean distance
            distance = np.sqrt((closest_asv_x - closest_point_x) ** 2 + (closest_asv_y - closest_point_y) ** 2) 
            
            if obs_id not in distances:
                distances[obs_id] = []
            distances[obs_id].append((relative_ts, distance))
    return distances


def plot_distances(distances_ground_truth, distances_detection, time_range=None):
    """Plot ASV distance to each obstacle over relative time from both ground truth and detection data. Returns the figure object."""
    fig = plt.figure(figsize=(5.5, 4))  
    
    # Create a color map for unique colors for each obstacle
    color_map = cm.get_cmap('tab10')  # Use the 'tab10' colormap for distinct colors
    
    # Plot distances for ground truth obstacles (segmented lines)
    for idx, (obs_id, dist_list) in enumerate(distances_ground_truth.items()):
        ts_vals, dist_vals = zip(*sorted(dist_list, key=lambda x: x[0]))
        color = color_map(idx % 10)  # Select a unique color for each obstacle
        plt.plot(ts_vals, dist_vals, linestyle='--', color=color, label=f"GT-{obs_id}")  # Segmented line for GT
        
        # Find the timestamp when the minimum distance occurs
        min_distance_idx = np.argmin(dist_vals)
        min_distance_time = ts_vals[min_distance_idx]
        
        # Plot a vertical line at the minimum distance for ground truth
        plt.axvline(x=min_distance_time, color=color, linestyle=':', label=f"GT Min {obs_id}")
    
    # Plot distances for detection obstacles (full lines)
    for idx, (obs_id, dist_list) in enumerate(distances_detection.items()):
        ts_vals, dist_vals = zip(*sorted(dist_list, key=lambda x: x[0]))
        color = color_map((idx + len(distances_ground_truth)) % 10)  # Use different set of colors for detection
        plt.plot(ts_vals, dist_vals, linestyle='-', color=color, label=f"DTC-{obs_id}")  # Full line for DTC
        
        # Find the timestamp when the minimum distance occurs
        min_distance_idx = np.argmin(dist_vals)
        min_distance_time = ts_vals[min_distance_idx]
        
        # Plot a vertical line at the minimum distance for detection
        plt.axvline(x=min_distance_time, color=color, linestyle='-.', label=f"DTC Min {obs_id}")
    
    plt.xlabel("Time (s)")
    plt.ylabel("Distance (m)")
    plt.title("ASV Distance to Obstacles Over Time")
    plt.legend(loc='upper right')
    plt.gca().set_facecolor('whitesmoke')
    plt.grid(True)
    
    # Optionally synchronize the x-axis with the full duration of the ASV trace
    if time_range:
        plt.xlim(time_range)
    
    plt.tight_layout()
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
    
    # Read ASV positions and timestamps from the bag file
    positions_x, positions_y, timestamps = read_positions_from_bag(bag_path)
    if not timestamps:
        print("No timestamp data available.")
        sys.exit(1)
    
    # Set start_time based on the first path timestamp if available, otherwise use first position timestamp
    paths = read_path_data(bag_path)
    if paths:
        start_time = min(paths.keys())
    else:
        start_time = timestamps[0]
    
    OBSTACLES_DTC_TOPIC = "/trk/obstacle_list"
    OBSTACLES_GT_TOPIC = "/gt/obstacle_list"
    obstacles_data_ground_truth = read_obstacles_from_bag(bag_path, OBSTACLES_GT_TOPIC)
    obstacles_data_detection = read_obstacles_from_bag(bag_path, OBSTACLES_DTC_TOPIC)
    
    # Compute distances for both ground truth and detection obstacles using the updated start_time
    distances_ground_truth = compute_distances(positions_x, positions_y, timestamps, obstacles_data_ground_truth, start_time)
    distances_detection = compute_distances(positions_x, positions_y, timestamps, obstacles_data_detection, start_time)
    
    closest_times = compute_closest_times(distances_ground_truth)
    
    # Plot both distances with x-axis relative to start_time (i.e., starting at 0s)
    time_min = 0.0
    time_max = max(timestamps) - start_time
    fig2 = plot_distances(distances_ground_truth, distances_detection, time_range=(time_min, time_max))
    

    
    # Pass timestamps along to plot_all so we can compare relative times
    fig1 = plot_all(positions_x, positions_y, paths, start_time, closest_times, timestamps)
    
    plt.show()

if __name__ == "__main__":
    main()
