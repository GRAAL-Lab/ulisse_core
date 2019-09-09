
from ulisse_msgs.srv import *
from ulisse_msgs.msg import ControlData
from pandas import DataFrame
import matplotlib.pyplot as plt
import numpy as np
import rclpy
import sys

import time
from std_msgs.msg import String

g_node = None

e = dict()
e['time'] = []
e['surge_error'] = []
e['yawr_error'] = []
e['surge_control'] = []
e['yawr_control'] = []

timer_reached = True

def clb(request,response):
    pass
    
def chatter_callback(msg):
    global e, t0, timer_reached
    t1 = time.time()
    e['time'].append(t1-t0) 
    e['surge_error'].append(msg.surge_error)
    e['yawr_error'].append(msg.yawr_error)
    e['surge_control'].append(msg.surge_control)
    e['yawr_control'].append(msg.yawr_control)
    if ((t1-t0) >= 40) :
        timer_reached = False

def main(args=None):
    global g_node, t0, e
    rclpy.init(args=args)

    g_node = rclpy.create_node('test_service')

    srv = g_node.create_client(ControlCommand, str(sys.argv[1]))

    while not srv.wait_for_service(timeout_sec=1.0):
        g_node.get_logger().info('service not available, waiting again...')

    req = ControlCommand.Request()
    req.command_type = "speedheading_command"
    req.sh_cmd.speed = 0.0 
    req.sh_cmd.heading = 90 * 3.14/180 
    future = srv.call_async(req)
    rclpy.spin_until_future_complete(g_node, future)

    subscription = g_node.create_subscription(ControlData, 'ulisse/ControlData', chatter_callback)
    subscription  # prevent unused variable warning

    t0 = time.time()
    while(rclpy.ok() and timer_reached):
        rclpy.spin_once(g_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    df = DataFrame(e, columns= ['time', 'surge_ref','yawr_ref', 'surge_out','yawr_out'])
    export_csv = df.to_csv (r'export_dataframe.csv', index = None, header=True)
    
    req.command_type = "speedheading_command"
    req.sh_cmd.speed = 0.0 
    req.sh_cmd.heading = 0.0* 3.14/180 
    future = srv.call_async(req)
    rclpy.spin_until_future_complete(g_node, future)

    g_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()