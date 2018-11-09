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

### Run the Ros1/Ros2 bridge and Rosbag recorder
Launch the ROS1 rosbag recorder:

    sourceros1
    roslaunch record_ros record_ros.launch

Then, in another terminal execute the bridge using script located in the ulisse_ctrl folder:

    ./ulisse_ctrl/scripts/run_ros_bridge.sh

Now using a service call in the `record/record_bag` topic you can start and stop the logger, for example:

    # ROS1
    sourceros1
    rosservice call /record/record_bag "cmd: 'start'"

    # ROS2
    sourceros2
    rosservice call /record/record_bag "cmd: 'start'"
