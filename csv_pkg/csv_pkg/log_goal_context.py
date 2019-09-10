
from ulisse_msgs.msg import GoalContext
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
e['goal_distance'] = []
e['goal_heading'] = []
e['goal_speed'] = []

timer_reached = True

def chatter_callback(msg):
    global e, t0, timer_reached
    e['time'].append(msg.stamp.sec + (msg.stamp.nanosec * 1e-9)) 
    e['goal_heading'].append(msg.goal_heading)
    e['goal_distance'].append(msg.goal_distance)
    e['goal_speed'].append(msg.goal_speed)

def stop_callback(request, response):
    global g_node
    global timer_reached
    g_node.get_logger().info('Incoming request\nStop Status Context Logger')

    timer_reached = False
    return response


def main(args=None):
    global g_node, t0, e
    rclpy.init(args=args)

    g_node = rclpy.create_node('logger_status_context')

    subscription = g_node.create_subscription(GoalContext, 'ulisse/ctrl/pos_context', chatter_callback)
    subscription  # prevent unused variable warning

    srv = g_node.create_service(Empty, 'ulisse/stop/goal_context', stop_callback)

    t0 = time.time()
    while(rclpy.ok() and timer_reached):
    	rclpy.spin_once(g_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

    df = DataFrame(e, columns= ['time', 'goal_heading', 'goal_speed', 'goal_distance'])
    
    directory = home + "/log_ulisse"
    if not os.path.isdir(directory):
        os.mkdir(directory)

    export_csv = df.to_csv (directory + '/log_goal_context.csv', index = None, header=True)

    g_node.destroy_service(srv)
    g_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()