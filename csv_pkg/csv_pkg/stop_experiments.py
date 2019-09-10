# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from std_srvs.srv import Empty

import rclpy


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('stop_experiments')

    clients = []
    clients.append(node.create_client(Empty, 'ulisse/stop/status_context'))
    clients.append(node.create_client(Empty, 'ulisse/stop/control_context'))
    clients.append(node.create_client(Empty, 'ulisse/stop/nav_filter'))
    clients.append(node.create_client(Empty, 'ulisse/stop/llc_gps_sensor'))
    clients.append(node.create_client(Empty, 'ulisse/stop/llc_imu_sensor'))
    clients.append(node.create_client(Empty, 'ulisse/stop/llc_magnetometer_sensor'))
    clients.append(node.create_client(Empty, 'ulisse/stop/llc_compass_sensor'))
    clients.append(node.create_client(Empty, 'ulisse/stop/generic'))
    clients.append(node.create_client(Empty, 'ulisse/stop/dcl_control'))
    clients.append(node.create_client(Empty, 'ulisse/stop/goal_context'))

    req = Empty.Request()

    for cli in clients:
        while not cli.wait_for_service(timeout_sec=1.0):
            node.get_logger().info('service not available, waiting again...')

        future = cli.call_async(req)
        rclpy.spin_until_future_complete(node, future)

    node.get_logger().info('Experiment Finished')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()