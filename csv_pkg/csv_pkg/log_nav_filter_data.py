
from ulisse_msgs.msg import NavFilterData
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
e['latitude'] = []
e['longitude'] = []
e['altitude'] = []
e['yaw'] = []
e['pitch'] = []
e['roll'] = []
e['accelerometer_x'] = []
e['accelerometer_y'] = []
e['accelerometer_z'] = []
e['gyro_x'] = []
e['gyro_y'] = []
e['gyro_z'] = []
e['speed_x'] = []
e['speed_y'] = []
e['current_x'] = []
e['current_y'] = []

timer_reached = True

def chatter_callback(msg):
    global e, timer_reached
    e['time'].append(msg.stamp.sec + (msg.stamp.nanosec * 1e-9)) 
    e['latitude'].append(msg.latitude)
    e['longitude'].append(msg.longitude)
    e['altitude'].append(msg.altitude)
    e['yaw'].append(msg.orientation.yaw)
    e['pitch'].append(msg.orientation.pitch)
    e['roll'].append(msg.orientation.roll)
    e['accelerometer_x'].append(msg.accelerometer[0])
    e['accelerometer_y'].append(msg.accelerometer[1])
    e['accelerometer_z'].append(msg.accelerometer[2])
    e['gyro_x'].append(msg.gyro[0])
    e['gyro_y'].append(msg.gyro[1])
    e['gyro_z'].append(msg.gyro[2])
    e['speed_x'].append(msg.speed[0])
    e['speed_y'].append(msg.speed[1])
    e['current_x'].append(msg.current[0])
    e['current_y'].append(msg.current[1])

def stop_callback(request, response):
    global g_node
    global timer_reached
    g_node.get_logger().info('Incoming request\nStop NAv Filter Logger')

    timer_reached = False
    return response


def main(args=None):
    global g_node, e
    rclpy.init(args=args)

    g_node = rclpy.create_node('logger_nav_filter')

    subscription = g_node.create_subscription(NavFilterData, 'ulisse/nav_filter/data', chatter_callback)
    subscription  # prevent unused variable warning

    srv = g_node.create_service(Empty, 'ulisse/stop/nav_filter', stop_callback)

    while(rclpy.ok() and timer_reached):
    	rclpy.spin_once(g_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

    df = DataFrame(e, columns= ['time', 'latitude', 'longitude', 'altitude', 'yaw', 'pitch', 'roll', 'accelerometer_x', 'accelerometer_y', 'accelerometer_z', 'gyro_x', 'gyro_y', 'gyro_z', 'speed_x', 'speed_y', 'current_x', 'current_y'])
   
    directory = home + "/log_ulisse"
    if not os.path.isdir(directory):
        os.mkdir(directory)
 
    export_csv = df.to_csv (directory + '/log_nav_filter.csv', index = None, header=True)

    g_node.destroy_service(srv)
    g_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()