# Ulisse Catamaran Control

Ulisse catamaran controller revamped with ROS2.

## Dependencies

- **rml**: http://bitbucket.org/isme_robotics/rml
- **fsm**: http://bitbucket.org/isme_robotics/fsm
- **ctrl_toolbox**: http://bitbucket.org/isme_robotics/ctrl_toolbox
- **libgeographic**: ` sudo apt install libgeographic-*`
- **libgps**: `sudo apt install libgps-dev`
- **qt5-libraries**: When building the package on the catamaran you don't need the **ulisse_map** sub-package, and this dependencies can be skipped. Otherwise, to use the interface, type: `sudo apt install qtquickcontrols2-5-dev qtlocation5-dev qtpositioning5-dev qml-module-qtquick-controls2 qml-module-qtlocation qml-module-qt-labs-* qml-module-qtpositioning qml-module-qtquick-extras qml-module-qtgraphicaleffects qml-module-qtquick-dialogs qml-module-qtquick-controls`

### Additional packages not coming with ros2

- **colcon**: Follow the guide at https://colcon.readthedocs.io/en/master/user/installation.html
- **rosbag2**: Follow the guide at https://github.com/ros2/rosbag2

## Subpackages

- **surface_vehicle_model**: Dynamic model of a surface vehicle with two thrusters.
- **ulisse_msgs**: Interface and services messages package with headers for topic names and common variables.
- **ulisse_core**: The catamaran controller.
- **ulisse_driver**: The low level driver that communicate.
- **ulisse_sim**: The dynamic simulator, which makes use of `surface_vehicle_model` library.
- **ulisse_map**: Graphical interface (Qt based) for controlling the catamaran.

## Build

Be sure to start from a clean workspace, with no _log_, _install_ or _build_ folders. First install the needed dependencies, and download the ros_bridge (from the ros official repo https://github.com/ros2/ros1_bridge). Navigate to your ROS1 workspace and build the `ulisse_rosbag_ros1` package (https://bitbucket.org/isme_robotics/ulisse_rosbag_ros1/):

```
#!bash
sourceros1
catkin_make
```

Then, navigate to your ROS2 workspace and execute the following commands to build this package:

```
#!bash
sourceros2
colcon build --symlink-install --packages-skip ros1_bridge
source /opt/ros/melodic/setup.bash
source ~/ros_ws/devel/setup.bash
source ~/ros2_ws/install/setup.bash 
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
```

*Note*: If you are building on a memory constrained system you might want to limit the number of parallel jobs by setting e.g. the environment variable `MAKEFLAGS=-j2` (more info on https://github.com/ros2/ros1_bridge/).

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
# Shell D
sourceros1
roslaunch prog_rosbag record_bag.launch
```

Then, in another terminal navigate to the repository root and execute the bridge using script located in the ulisse_ctrl folder:

```
#!bash
# Shell E
./ulisse_ctrl/scripts/run_ros_bridge.sh
```

Now using a service call in the `/record_bag` topic you can start and stop the logger, via C++ API or terminal. For example, using the ROS terminal interface:

```
#!bash
# Shell F: ROS1
sourceros1
rosservice call /record_bag "cmd: 'start'"
rosservice call /record_bag "cmd: 'stop'"

# Shell F: ROS2
sourceros2
ros2 service call /record_bag ulisse_msgs/RosbagCmd 'cmd: start'
ros2 service call /record_bag ulisse_msgs/RosbagCmd 'cmd: stop'
```

### Testing the serial

Be sure that in the driver configuration file *ulisse_driver/conf/ulisse_driver.yaml* the **SerialDevice** paramater is set to `"/tmp/serial1"`. Then, run the following commands in three separate ROS2 sourced terminals (`sourceros2` command):

```
#!bash
# Shell A (setup serial)
socat -d -d pty,raw,echo=0,link=/tmp/serial1 pty,raw,echo=0,link=/tmp/serial2

# Shell B (launch driver)
ros2 launch ulisse_driver launchDriver.py

# Shell C (echo on any topic of interest, e,g. /sensor/ambient)
ros2 topic echo /sensor/ambient

# Shell D (transmit data on serial)
cat ~/graal_ws/serialS0.txt > /tmp/serial2
```

## Misc

For additional info look [info.txt](./info.txt).
