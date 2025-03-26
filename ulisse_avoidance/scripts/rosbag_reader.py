# rosbag_reader.py
import rosbag2_py
import rosbag2_py
import sys
import os
import glob
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from conversion import latlong_to_ned


ASV_POSE_TOPIC = "/ulisse/simulated_system"
ASV_PATH_TOPIC = "/ulisse/avoidance/current_path"

def get_bag_by_index(directory, index=1):
    """Finds the bag file by index, where 1 is the most recent, 2 is the second most recent, etc."""
    # Look for folders that start with 'rosbag2_'
    bag_folders = glob.glob(os.path.join(directory, 'rosbag2_*'))

    if not bag_folders:
        print("No bag folders found in the directory.")
        return None

    # Sort the folders by modification time, latest first
    sorted_folders = sorted(bag_folders, key=os.path.getmtime, reverse=True)

    # Select the folder based on the index
    if index <= len(sorted_folders):
        latest_folder = sorted_folders[index - 1]
    else:
        print(f"Invalid index {index}. Only {len(sorted_folders)} bag folders found.")
        return None

    # Now look for the .db3 file inside the selected folder
    db3_files = glob.glob(os.path.join(latest_folder, '*.db3'))

    if not db3_files:
        print(f"No .db3 file found in the folder {latest_folder}.")
        return None

    # Return the most recent .db3 file
    return max(db3_files, key=os.path.getmtime)

def get_most_recent_bag(directory):
    """Finds the most recent .db3 bag file in the given directory."""
    # Look for folders that start with 'rosbag2_'
    bag_folders = glob.glob(os.path.join(directory, 'rosbag2_*'))

    if not bag_folders:
        print("No bag folders found in the directory.")
        return None

    # Get the most recent folder by modification time
    latest_folder = max(bag_folders, key=os.path.getmtime)

    # Now look for the .db3 file inside the latest folder
    db3_files = glob.glob(os.path.join(latest_folder, '*.db3'))

    if not db3_files:
        print(f"No .db3 file found in the folder {latest_folder}.")
        return None

    # Return the most recent .db3 file
    return max(db3_files, key=os.path.getmtime)



def ROSStampToTs(stamp):
    return stamp.sec + stamp.nanosec * 1e-9



def ROSStampToTs(stamp):
    """Convert ROS2 timestamp to a floating-point timestamp in seconds."""
    return stamp.sec + stamp.nanosec * 1e-9

def read_obstacles_from_bag(bagPath: str, topicName: str):
    try:
        storage_options = rosbag2_py.StorageOptions(uri=bagPath, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Error opening the bag: {e}")
        sys.exit(1)

    topic_type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topicName not in topic_type_map:
        print(f"Topic '{topicName}' not found in the bag.")
        sys.exit(1)

    message_type = get_message(topic_type_map[topicName])

    print(f"Reading messages on topic '{topicName}' from bag '{bagPath}':")

    obstacles_by_time = {}  # {timestamp: {obstacle_id: obstacle_data}}

    while reader.has_next():
        topic, serialized_data, _ = reader.read_next()
        if topic == topicName:
            msg = deserialize_message(serialized_data, message_type)
            ts = ROSStampToTs(msg.header.stamp)  # Get timestamp from ROS message
            
            obstacles = {}  # Store obstacles for this timestamp
            for obs in msg.obstacles:
                obs_id = obs.id
                obs_class = obs.obs_class
                local_x, local_y = latlong_to_ned(obs.pose.position.position.latitude, obs.pose.position.position.longitude)
                yaw = obs.pose.orientation.yaw
                size_x, size_y = obs.size.size.length, obs.size.size.width

                obstacles[obs_id] = {
                    "class": obs_class,
                    "position": (local_x, local_y),
                    "yaw": yaw,
                    "size": (size_x, size_y),
                }

            # Store the obstacles with their timestamp
            obstacles_by_time[ts] = obstacles

    return obstacles_by_time  # Dictionary {timestamp: {id: obstacle_data}}


def read_positions_from_bag(bagPath: str, topicName: str = ASV_POSE_TOPIC):
    try:
        storage_options = rosbag2_py.StorageOptions(uri=bagPath, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Error opening the bag: {e}")
        sys.exit(1)

    topic_type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topicName not in topic_type_map:
        print(f"Topic '{topicName}' not found in the bag.")
        sys.exit(1)

    message_type = get_message(topic_type_map[topicName])

    print(f"Reading messages on topic '{topicName}' from bag '{bagPath}':")
    positions_x = []
    positions_y = []
    timestamps = []

    # Iterate over messages in the bag
    while reader.has_next():
        topic, serialized_data, _ = reader.read_next()
        if topic == topicName:
            msg = deserialize_message(serialized_data, message_type)
            ts = ROSStampToTs(msg.stamp)
            pos = msg.inertialframe_linear_position
            lat, lon = pos.latlong.latitude, pos.latlong.longitude

            local_x, local_y= latlong_to_ned(lat, lon)
            positions_x.append(local_x)
            positions_y.append(local_y)
            timestamps.append(ts)

    return positions_x, positions_y, timestamps

def read_path_data(bag_path: str, topicName: str = ASV_PATH_TOPIC):
    """Reads multiple path instances from the rosbag and extracts coordinates, centroid, and velocities."""
    try:
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Error opening bag: {e}")
        return {}

    topic_type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topicName not in topic_type_map:
        print("Path topic not found in bag.")
        return {}

    message_type = get_message(topic_type_map[topicName])

    print(f"Reading messages on topic '{topicName}' from bag '{bag_path}':")
    paths = {}

    while reader.has_next():
        topic, serialized_data, _ = reader.read_next()
        if topic == topicName:
            msg = deserialize_message(serialized_data, message_type)
            ts = ROSStampToTs(msg.stamp)  # Convert ROS timestamp to readable format

            # Convert path to NED
            path_coordinates = [latlong_to_ned(p.latitude, p.longitude) for p in msg.path.coordinates]
            path_velocities = msg.path.velocities  # Array of velocities
            path_velocities_abscissas = msg.path.velocities_abscissas  # Array of abscissas (distances along path)

            if ts not in paths:
                # Store the path data along with velocities and abscissas
                paths[ts] = {
                    "coordinates": path_coordinates,
                    "velocities": path_velocities,
                }
            else:
                print(f"Duplicate timestamp found for {ts}. Overwriting path.")

    return paths  # Dictionary {timestamp: {path_data}}



