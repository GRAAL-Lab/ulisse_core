
from ulisse_msgs.msg import TaskStatus
from pandas import DataFrame
import numpy as np
import rclpy, os

import os
import time

from std_msgs.msg import String

from std_srvs.srv import Empty

g_node = None

from os.path import expanduser

home = expanduser("~")

e = dict()
e['time'] = []
e['activation_function'] = []
e['is_active'] = []
e['reference'] = []

timer_reached = True

def chatter_callback(msg):
    global e
    e['time'].append(msg.stamp.sec + (msg.stamp.nanosec * 1e-9)) 
    e['activation_function'].append(msg.activation_function)
    e['is_active'].append(msg.is_active)
    e['reference'].append(msg.reference)

def stop_callback(request, response):
    global g_node
    global timer_reached
    g_node.get_logger().info('Incoming request\nStop Status Context Logger')

    timer_reached = False
    return response


def main(args=None):
    global g_node, t0, e
    rclpy.init(args=args)
    g_node = rclpy.create_node('logger_asv_control_distance')

    subscription = g_node.create_subscription(TaskStatus, 'ulisse/log/task/asv_safety_boundaries', chatter_callback)
    subscription  # prevent unused variable warning

    srv = g_node.create_service(Empty, 'ulisse/stop/task/asv_safety_boundaries', stop_callback)

    t0 = time.time()
    while(rclpy.ok() and timer_reached):
    	rclpy.spin_once(g_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

    df = DataFrame(e, columns= ['time', 'activation_function','is_active', 'reference'])
    
    directory = home + "/log_ulisse"
    if not os.path.isdir(directory):
        os.mkdir(directory)

    export_csv = df.to_csv (directory + '/log_task_asv_safety_boundaries.csv', index = None, header=True)

    g_node.destroy_service(srv)
    g_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()