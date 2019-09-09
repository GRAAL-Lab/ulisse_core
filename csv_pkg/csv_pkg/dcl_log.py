
from ulisse_msgs.srv import *
from ulisse_msgs.msg import *
from pandas import DataFrame
import matplotlib.pyplot as plt
import numpy as np
import rclpy
import sys
import os
import time
from std_msgs.msg import String
from os.path import expanduser

from std_srvs.srv import Empty

g_node = None

e = dict()
e['time'] = []
e['surge_error'] = []
e['yawr_error'] = []
e['surge_control'] = []
e['yawr_control'] = []
e['thruster_map_right'] = []
e['thruster_map_left'] = []
e['thruster_right'] = []
e['thruster_left'] = []

timer_reached = True
home = expanduser("~")

def clb(request,response):
    pass
    
def chatter_callback(msg):
    global e, timer_reached
    e['time'].append(msg.stamp.sec + (msg.stamp.nanosec * 1e-9)) 
    e['surge_error'].append(msg.surge_error)
    e['yawr_error'].append(msg.yawr_error)
    e['surge_control'].append(msg.surge_control)
    e['yawr_control'].append(msg.yawr_control)
    e['thruster_right'].append(msg.thrust_right)
    e['thruster_left'].append(msg.thrust_left)

def cb(msg):
    global e
    e['thruster_map_right'].append(msg.motor_ctrlref.right)
    e['thruster_map_left'].append(msg.motor_ctrlref.left)

def stop_callback(request, response):
    global g_node
    global timer_reached
    g_node.get_logger().info('Incoming request\nStop Control Context Logger')

    timer_reached = False
    return response

def main(args=None):
    global g_node, e
    rclpy.init(args=args)

    g_node = rclpy.create_node('test_service')

    srv_ = g_node.create_service(Empty, 'ulisse/stop/dcl_control', stop_callback)

    subscription1 = g_node.create_subscription(ThrustersData, "ulisse/ctrl/thruster_ref", cb )
    subscription = g_node.create_subscription(ControlData, 'ulisse/ControlData', chatter_callback)
    subscription  # prevent unused variable warning

    while(rclpy.ok() and timer_reached):
        rclpy.spin_once(g_node)
    
    directory = home + "/log_ulisse"
    if not os.path.isdir(directory):
        os.mkdir(directory)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    df = DataFrame(e, columns= ['time', 'surge_ref','yawr_ref', 'surge_out','yawr_out','thruster_map_right','thruster_map_left','thruster_right','thruster_left'])
    export_csv = df.to_csv (directory + '/export_dataframe.csv', index = None, header=True)
    
    g_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()