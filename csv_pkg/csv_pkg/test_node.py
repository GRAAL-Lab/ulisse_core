
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
home = expanduser.("~")

def stop_callback(request, response):
    global g_node
    global timer_reached
    g_node.get_logger().info('Incoming request\nStop Control Context Logger')

    timer_reached = False
    return response
    
def chatter_callback(msg):
    global e, t0, timer_reached
    t1 = time.time()
    e['time'].append(msg.stamp.sec + (msg.stamp.nanosec * 1e-9)) 
    e['surge_error'].append(msg.surge_error)
    e['yawr_error'].append(msg.yawr_error)
    e['surge_control'].append(msg.surge_control)
    e['yawr_control'].append(msg.yawr_control)
    e['thruster_right'].append(msg.thrust_right)
    e['thruster_left'].append(msg.thrust_left)
    e['thruster_map_right'].append(msg.thrust_map_right)
    e['thruster_map_left'].append(msg.thrust_map_left)

    if ((t1-t0) >= 40) :
        timer_reached = False

def cb(msg):
    global e

def main(args=None):
    global g_node, t0, e
    rclpy.init(args=args)

    g_node = rclpy.create_node('test_service')

    srv = g_node.create_service(Empty, 'ulisse/stop/control_context', stop_callback)

    while not srv.wait_for_service(timeout_sec=1.0):
        g_node.get_logger().info('service not available, waiting again...')

    req = ControlCommand.Request()
    req.command_type = "speedheading_command"
    req.sh_cmd.speed = float(sys.argv[1])
    req.sh_cmd.heading = float(sys.argv[2])* 3.14/180 
    future = srv.call_async(req)
    rclpy.spin_until_future_complete(g_node, future)

    subscription1 = g_node.create_subscription(ThrustersData, "ulisse/ctrl/thruster_ref", cb )
    subscription = g_node.create_subscription(ControlData, 'ulisse/ControlData', chatter_callback)
    subscription  # prevent unused variable warning

    t0 = time.time()
    while(rclpy.ok() and timer_reached):
        rclpy.spin_once(g_node)

    
    directory = home + "/log_ulisse"
    if not os.path.isdir(directory):
        os.mkdir(directory)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    df = DataFrame(e, columns= ['time', 'surge_ref','yawr_ref', 'surge_out','yawr_out','thruster_right','thruster_left','thruster_map_right','thruster_map_left'])
    export_csv = df.to_csv (directory + '/export_dataframe.csv', index = None, header=True)
    
    req.command_type = "speedheading_command"
    req.sh_cmd.speed = 0.0 
    req.sh_cmd.heading = 0.0* 3.14/180 
    future = srv.call_async(req)
    rclpy.spin_until_future_complete(g_node, future)
    
    g_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()