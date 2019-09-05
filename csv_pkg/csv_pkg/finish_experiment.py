import time

g_node = None

from os.path import expanduser

home = expanduser("~")

def main(args=None):
    global g_node, t0, e
    rclpy.init(args=args)

    g_node = rclpy.create_node('logger_generic_messages')

    g_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()