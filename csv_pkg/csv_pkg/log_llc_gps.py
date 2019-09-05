
from ulisse_msgs.msg import GPSData
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
e['track'] = []
e['speed'] = []
e['err'] = []
e['err_latitude'] = []
e['err_longitude'] = []
e['err_track'] = []
e['err_speed'] = []
e['err_climb'] = []

timer_reached = True

def chatter_callback(msg):
    global e, timer_reached
    e['time'].append(msg.time) 
    e['latitude'].append(msg.latitude)
    e['longitude'].append(msg.longitude)
    e['altitude'].append(msg.altitude)
    e['track'].append(msg.track)
    e['speed'].append(msg.speed)
    e['err'].append(msg.err)
    e['err_latitude'].append(msg.err_latitude)
    e['err_longitude'].append(msg.err_longitude)
    e['err_track'].append(msg.err_track)
    e['err_speed'].append(msg.err_speed)
    e['err_climb'].append(msg.err_climb)

def stop_callback(request, response):
    global g_node
    global timer_reached
    g_node.get_logger().info('Incoming request\nStop GPS Logger')

    timer_reached = False
    return response


def main(args=None):
    global g_node, e
    rclpy.init(args=args)

    g_node = rclpy.create_node('logger_llc_gps_sensor')

    subscription = g_node.create_subscription(GPSData, 'ulisse/llc/sensor/gps_data', chatter_callback)
    subscription  # prevent unused variable warning

    srv = g_node.create_service(Empty, 'ulisse/stop/llc_gps_sensor', stop_callback)

    while(rclpy.ok() and timer_reached):
    	rclpy.spin_once(g_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    df = DataFrame(e, columns= ['time', 'latitude','longitude', 'altitude','track', 'speed', 'err', 'err_latitude', 'err_longitude', 'err_track', 'err_speed', 'err_climb'])
    
    directory = home + "/log_ulisse"
    if not os.path.isdir(directory):
        os.mkdir(directory)

    export_csv = df.to_csv (directory + '/log_llc_gps.csv', index = None, header=True)

    g_node.destroy_service(srv)
    g_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()