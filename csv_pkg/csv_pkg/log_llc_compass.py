
from ulisse_msgs.msg import Compass
from pandas import DataFrame
import numpy as np
import rclpy

import os
import time

from std_msgs.msg import String

from std_srvs.srv import Empty

g_node = None

from os.path import expanduser

home = expanduser("~")

e = dict()
e['time'] = []
e['yaw'] = []
e['pitch'] = []
e['roll'] = []

timer_reached = True

def chatter_callback(msg):
    global e, timer_reached
    e['time'].append(msg.stamp.sec + (msg.stamp.nanosec * 1e-9)) 
    e['yaw'].append(msg.orientation.yaw)
    e['pitch'].append(msg.orientation.pitch)
    e['roll'].append(msg.orientation.roll)

def stop_callback(request, response):
    global g_node
    global timer_reached
    g_node.get_logger().info('Incoming request\nStop Compass Logger')

    timer_reached = False
    return response


def main(args=None):
    global g_node, e
    rclpy.init(args=args)

    g_node = rclpy.create_node('logger_llc_compass_sensor')

    subscription = g_node.create_subscription(Compass, 'ulisse/llc/sensor/compass', chatter_callback)
    subscription  # prevent unused variable warning

    srv = g_node.create_service(Empty, 'ulisse/stop/llc_compass_sensor', stop_callback)

    while(rclpy.ok() and timer_reached):
    	rclpy.spin_once(g_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    df = DataFrame(e, columns= ['time', 'yaw','pitch', 'roll'])
    
    directory = home + "/log_ulisse"
    if not os.path.isdir(directory):
        os.mkdir(directory)

    export_csv = df.to_csv (directory + '/log_llc_compass.csv', index = None, header=True)

    g_node.destroy_service(srv)
    g_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()