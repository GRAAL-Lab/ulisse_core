# Ulisse Catamaran Control

Ulisse catamaran controller revamped with ROS2.

## Dependencies

- **rml**: http://bitbucket.org/isme_robotics/rml
- **fsm**: http://bitbucket.org/isme_robotics/fsm
- **ctrl_toolbox**: http://bitbucket.org/isme_robotics/ctrl_toolbox
- **libgeographic**: ` sudo apt install libgeographic-*`
- **libgps**: `sudo apt install libgps-dev`


### ulisse_msgs

Interface messages package.


### ulisse_core

The actual controller.


### ulisse_driver

The low level driver.

### ulisse_sim

The simulator.

## BUILD

Be sure to start from a clean workspace, with no _log_, _install_ or _build_ folders. First install the needed dependencies, then to build the package and the ros_bridge (which has to be downloaded from the ros official repo https://github.com/ros2/ros1_bridge) use the following commands:

     sourceros2
     colcon build --symlink-install --packages-skip ros1_bridge
     sourceros1 && sourceros2
     colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure

## USAGE

First of all you need to export the ROS1 master URI for all your bashes, so it can be convenient to add this line to your **~/.bashrc**: `export ROS_MASTER_URI=http://localhost:11311/`.

### Run the Ros1/Ros2 bridge and Rosbag recorder
Launch the ROS1 rosbag recorder (repository https://bitbucket.org/isme_robotics/ulisse_rosbag_ros1):

    # Shell A
    sourceros1
    roslaunch prog_rosbag record_bag.launch

Then, in another terminal execute the bridge using script located in the ulisse_ctrl folder:

    # Shell B
    ./ulisse_ctrl/scripts/run_ros_bridge.sh

Now using a service call in the `/record_bag` topic you can start and stop the logger, via C++ API or terminal. For example, using the ROS terminal interface:

    # Shell C: ROS1
    sourceros1
    rosservice call /record_bag "cmd: 'start'"
    rosservice call /record_bag "cmd: 'stop'"


    # Shell C: ROS2
    sourceros2
    ros2 service call /record_bag ulisse_msgs/RosbagCmd 'cmd: start'
    ros2 service call /record_bag ulisse_msgs/RosbagCmd 'cmd: stop'
