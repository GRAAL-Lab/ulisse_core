import rclpy
from rclpy.node import Node
import math
from time import sleep
import subprocess
import sys  # To handle command line arguments
from conversion import latlong_to_ned, ned_to_latlong
from ulisse_msgs.srv import ControlCommand, ComputeAvoidancePath
from ulisse_msgs.msg import CommandLatLong, LatLong, SurgeHeading
from builtin_interfaces.msg import Time


class UlisseNavigator(Node):
    def __init__(self):
        super().__init__('ulisse_navigator')

        # Create service clients
        self.surge_client = self.create_client(ControlCommand, '/ulisse/service/control_cmd')
        self.avoidance_client = self.create_client(ComputeAvoidancePath, '/ulisse/service/compute_avoidance_path')

        # Create publisher for SurgeHeading message
        self.surge_publisher = self.create_publisher(SurgeHeading, '/ulisse/ctrl/surge_heading', 10)

        # Wait for services
        self.wait_for_service(self.surge_client, "/ulisse/service/control_cmd")
        self.wait_for_service(self.avoidance_client, "/ulisse/service/compute_avoidance_path")

    def wait_for_service(self, client, service_name):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {service_name}...')

    def compute_heading(self, goal_north, goal_east):
        """ Compute heading angle (radians) to face the goal in NED (0,0). """
        return math.atan2(goal_east, goal_north)

    def send_surge_heading_command(self):
        """ Send heading command and publish heading value periodically. """
        # Send the heading command
        request = ControlCommand.Request()
        request.command_type = "surgeheading_command"

        self.get_logger().info(f"Sending Heading Command: heading={self.heading:.2f} rad")

        future = self.surge_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(f'Service Response: {future.result()}')
        else:
            self.get_logger().error('Heading command failed')

        # Start publishing the SurgeHeading message periodically for `n` seconds
        self.publish_heading_for_duration(6)  # Publish for 10 seconds (or change as needed)

    def publish_heading_for_duration(self, duration_seconds):
        """ Publish the SurgeHeading message for the given duration (in seconds). """
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration_seconds:
            surge_heading_msg = SurgeHeading()

            surge_heading_msg.surge = 0.0  # Placeholder surge value
            surge_heading_msg.heading = self.heading  # Use computed heading value

            # Publish the message to /ulisse/ctrl/surge_heading topic
            self.surge_publisher.publish(surge_heading_msg)
            self.get_logger().info(f"Published SurgeHeading: surge={surge_heading_msg.surge}, heading={surge_heading_msg.heading}")

            sleep(1)  # Wait 1 second before publishing the next heading

    def send_avoidance_path_command(self, latitude, longitude, radius, speed, colregs):
        """ Send avoidance path computation request. """
        request = ComputeAvoidancePath.Request()
        request.latlong_cmd = CommandLatLong()
        request.latlong_cmd.goal = LatLong(latitude=latitude, longitude=longitude)
        request.latlong_cmd.acceptance_radius = radius
        request.latlong_cmd.ref_speed = speed
        request.colregs_compliant = bool(colregs)

        self.get_logger().info(f"Computing Avoidance Path to lat={latitude}, lon={longitude}")

        future = self.avoidance_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(f'Avoidance Path Response: {future.result()}')
        else:
            self.get_logger().error('Avoidance path computation failed')

def main():
    # Check if we received command-line arguments
    detection_flag = '-detection' in sys.argv
    config_file = None

    # Get the path to the configuration file from the arguments
    if len(sys.argv) > 1:
        config_file = sys.argv[1]  # We assume the first argument after the script is the path to the config file

    if config_file is None:
        print("Please provide a configuration file as an argument (e.g., 'oceans_test/multi/multi_crossing.cfg').")
        sys.exit(1)

    rclpy.init()
    node = UlisseNavigator()

    # Ask user for goal in NED
    goal_north = float(input("Enter goal North (m): "))
    goal_east = float(input("Enter goal East (m): "))

    # Compute heading
    node.heading = node.compute_heading(goal_north, goal_east)

    # Convert NED to Lat/Long
    goal_lat, goal_lon = ned_to_latlong(goal_east, goal_north)

    # Send heading command and start publishing heading for 10 seconds
    node.send_surge_heading_command()

    node.send_avoidance_path_command(goal_lat, goal_lon, radius=5.0, speed=2.1, colregs=1)

    # Sleep before calling the next command
    sleep(5)

    # If detection flag is set, execute the ros2_obstacle_publisher command
    if detection_flag:
        command = f"ros2 run ros2_obstacle_publisher publisher {config_file} -stonefish -gt -detection"
    else:
        command = f"ros2 run ros2_obstacle_publisher publisher {config_file} -stonefish -gt"
        
    print(f"Running command: {command}")
    subprocess.run(command, shell=True)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
