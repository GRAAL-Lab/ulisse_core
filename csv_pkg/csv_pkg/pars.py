
from ulisse_msgs.msg import ControlData
from pandas import DataFrame
import matplotlib.pyplot as plt
import numpy as np
import rclpy

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

def chatter_callback(msg):
    global e, t0, timer_reached
    t1 = time.time()
    e['time'].append(t1-t0) 
    e['surge_error'].append(msg.surge_error)
    e['yawr_error'].append(msg.yawr_error)
    e['surge_control'].append(msg.surge_control)
    e['yawr_control'].append(msg.yawr_control)
    if ((t1-t0) >= 100) :
    	timer_reached = False

def main(args=None):
    global g_node, t0, e
    rclpy.init(args=args)

    g_node = rclpy.create_node('minimal_subscriber')

    subscription = g_node.create_subscription(ControlData, 'ControlData', chatter_callback)
    subscription  # prevent unused variable warning

    t0 = time.time()
    while(rclpy.ok() and timer_reached):
    	rclpy.spin_once(g_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    df = DataFrame(e, columns= ['time', 'surge_error','yawr_error', 'surge_control','yawr_control'])
    export_csv = df.to_csv (r'export_dataframe.csv', index = None, header=True)
    plt.figure()
    plt.subplot(221)
    plt.plot(e['time'], e['surge_error'], 'r-')
    plt.grid(True)
    plt.subplot(222)
    plt.plot(e['time'], e['yawr_error'], 'b-')
    plt.grid(True)
    plt.subplot(223)
    plt.plot(e['time'], e['surge_control'], 'k-')
    plt.grid(True)
    plt.subplot(224)
    plt.plot(e['time'], e['yawr_control'], 'g-')
    plt.grid(True)
    plt.show()
    g_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()