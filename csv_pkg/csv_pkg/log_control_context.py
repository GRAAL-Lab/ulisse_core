
from ulisse_msgs.msg import ControlContext
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

e = dict()
e['time'] = []
e['desired_speed'] = []
e['desired_jog'] = []

timer_reached = True

def chatter_callback(msg):
    global e, t0, timer_reached
    t1 = time.time()
    e['time'].append(t1-t0) 
    e['desired_speed'].append(msg.desired_speed)
    e['desired_jog'].append(msg.desired_jog)

def stop_callback(request, response):
    global g_node
    global timer_reached
    g_node.get_logger().info('Incoming request\nStop Control Context Logger')

    timer_reached = False
    return response


def main(args=None):
    global g_node, t0, e
    rclpy.init(args=args)

    g_node = rclpy.create_node('logger_control_context')

    subscription = g_node.create_subscription(ControlContext, 'ulisse/ctrl/ctrl_context', chatter_callback)
    subscription  # prevent unused variable warning

    srv = g_node.create_service(Empty, 'ulisse/stop/control_context', stop_callback)

    t0 = time.time()
    while(rclpy.ok() and timer_reached):
    	rclpy.spin_once(g_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

    df = DataFrame(e, columns= ['time', 'desired_speed','desired_jog'])
    export_csv = df.to_csv (home + '/log_control_context.csv', index = None, header=True)

    g_node.destroy_service(srv)
    g_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()