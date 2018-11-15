# Ulisse Catamaran Control

Ulisse catamaran controller revamped with ROS2.

## Dependencies

- **rml**: http://bitbucket.org/isme_robotics/rml
- **fsm**: http://bitbucket.org/isme_robotics/fsm
- **ctrl_toolbox**: http://bitbucket.org/isme_robotics/ctrl_toolbox
- **libgeographic**: ` sudo apt install libgeographic-*`
- **libgps**: `sudo apt install libgps-dev`


## Subpackages

- **surface_vehicle_model**: Dynamic model of a surface vehicle with two thrusters.
- **ulisse_msgs**: Interface and services messages package with headers for topic names and common variables.
- **ulisse_core**: The catamaran controller.
- **ulisse_driver**: The low level driver that communicate.
- **ulisse_sim**: The dynamic simulator, which makes use of `surface_vehicle_model` library.

## Build

Be sure to start from a clean workspace, with no _log_, _install_ or _build_ folders. First install the needed dependencies, then to build the package and the ros_bridge (which has to be downloaded from the ros official repo https://github.com/ros2/ros1_bridge) use the following commands:

```
#!bash
sourceros2
colcon build --symlink-install --packages-skip ros1_bridge
sourceros1 && sourceros2
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
```

## Usage

First of all you need to export the ROS1 master URI for all your bashes, so it can be convenient to add this line to your **~/.bashrc**: `export ROS_MASTER_URI=http://localhost:11311/`.

### Run the architecture
The following launch files will run all the necessary nodes (all with `sourceros2`):
```
#!bash
# Shell A (driver)
ros2 launch ulisse_driver launchDriver.py  # real case
-or-
ros2 launch ulisse_sim launchSim.py        # simulating

# Shell B (controller)
ros2 launch ulisse_ctrl launchControl.py

# Shell C (data monitor)
ros2 run ulisse_ctrl monitor_node
```

### Run the Ros1/Ros2 bridge and Rosbag recorder
Launch the ROS1 rosbag recorder (repository https://bitbucket.org/isme_robotics/ulisse_rosbag_ros1, in which you will also find the BAG to CSV converter):

```
#!bash
# Shell A
sourceros1
roslaunch prog_rosbag record_bag.launch
```

Then, in another terminal execute the bridge using script located in the ulisse_ctrl folder:

```
#!bash
# Shell B
./ulisse_ctrl/scripts/run_ros_bridge.sh
```

Now using a service call in the `/record_bag` topic you can start and stop the logger, via C++ API or terminal. For example, using the ROS terminal interface:

```
#!bash
# Shell C: ROS1
sourceros1
rosservice call /record_bag "cmd: 'start'"
rosservice call /record_bag "cmd: 'stop'"

# Shell C: ROS2
sourceros2
ros2 service call /record_bag ulisse_msgs/RosbagCmd 'cmd: start'
ros2 service call /record_bag ulisse_msgs/RosbagCmd 'cmd: stop'
```

### Testing the serial

Run the following commands in separate ROS2 sourced terminals (`sourceros2` command):

```
#!bash
# Shell A (setup serial)
socat -d -d pty,raw,echo=0 pty,raw,echo=0

# Shell B (launch driver)
ros2 launch ulisse_driver launchDriver.py

# Shell C (echo on any topic of interest)
cat ~/graal_ws/serialS0.txt > /dev/pts/6
```

## Misc

For additional info look [info.txt](./info.txt).
