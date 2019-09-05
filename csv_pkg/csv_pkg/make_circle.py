
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
go = False

timer_reached = True

def chatter_callback(msg):
    global latitude, longitude
    global go 
    go = True
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

    global go 
    while not go:
        rclpy.spin_once(g_node)
        
    radius = str(1)
    req = ControlCommand.Request()
    req.command_type = "navigate_command"
    req.nav_cmd.nurbs_json = "{\"centroid\":[" + str(latitude) +"," + str(longitude) + "],\"curves\":[{\"degree\":2,\"knots\":[0,0,0,0.25,0.25,0.5,0.5,0.75,0.75,1,1,1],\"points\":[[0,-" + radius + "],[-" + radius + ",-" + radius + "]," +  "[-" + radius + ",0],[-" + radius + "," + radius + "],[0," + radius + "],[" + radius + "," + radius + "],[" + radius + ",0],[" + radius + ",-" + radius + "]," + "[0,-" + radius + "]],\"weigths\":[1,0.707,1,0.707,1,0.707,1,0.707,1]}],\"direction\":0}"

    while not clients.wait_for_service(timeout_sec=1.0):
        g_node.get_logger().info('service not available, waiting again...')

    future = clients.call_async(req)
    rclpy.spin_until_future_complete(g_node, future)

    g_node.get_logger().info('Make Circle Finished')

    g_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()