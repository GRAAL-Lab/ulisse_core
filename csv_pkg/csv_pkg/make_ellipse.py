
from ulisse_msgs.msg import NavFilterData
from ulisse_msgs.srv import ControlCommand
from pandas import DataFrame
import matplotlib.pyplot as plt
import numpy as np
import rclpy

import time

from std_msgs.msg import String

from std_srvs.srv import Empty

g_node = None

from os.path import expanduser

home = expanduser("~")

latitude = 0.0
longitude = 0.0

timer_reached = True

def chatter_callback(msg):
    global latitude, longitude
    latitude = msg.latitude
    longitude = msg.longitude


def main(args=None):
    global g_node
    global latitude, longitude
    rclpy.init(args=args)

    g_node = rclpy.create_node('make_curve_from_spot')

    subscription = g_node.create_subscription(NavFilterData, 'ulisse/nav_filter/data', chatter_callback)
    subscription  # prevent unused variable warning

    clients = g_node.create_client(ControlCommand, '/ulisse/service/control_cmd')

    req = ControlCommand.Request()
    req.nav_cmd.nurbs_json = ""

    while not clients.wait_for_service(timeout_sec=1.0):
        g_node.get_logger().info('service not available, waiting again...')

    future = clients.call_async(req)
    rclpy.spin_until_future_complete(g_node, future)

    g_node.get_logger().info('Make Circle Finished')

    g_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()