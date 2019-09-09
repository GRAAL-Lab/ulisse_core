# Ulisse Catamaran Control

Ulisse catamaran controller revamped with ROS2.

## Subpackages

Short description of all the packages included in this meta package:

- **surface_vehicle_model**: Dynamic model of a surface vehicle with two thrusters.
- **navigation_filter**: The sensor filtering package providing the vehicle status and seacurrent estimation to the controller.
- **ulisse_msgs**: Interface and services messages package with headers for topic names and common variables.
- **ulisse_ctrl**: The catamaran controller.
- **ulisse_driver**: The low level driver that communicates with the catamaran microcontrollers.
- **ulisse_sim**: The dynamic simulator, which makes use of `surface_vehicle_model` library.
- **ulisse_map**: Graphical interface (Qt based) for controlling the catamaran.

## Dependencies

- **rml**: http://bitbucket.org/isme_robotics/rml
- **fsm**: http://bitbucket.org/isme_robotics/fsm
- **ctrl_toolbox**: http://bitbucket.org/isme_robotics/ctrl_toolbox
- **tpiklib**: http://bitbucket.org/isme_robotics/tpiklib
- **ikcl**: http://bitbucket.org/isme_robotics/ikcl
- **libgeographic**: `sudo apt install libgeographic-*`
- **libgps**: `sudo apt install libgps-dev`
- **libconfig++-dev**: `sudo apt install libconfig++-dev` 
- **libz3-dev**: `sudo apt install libz3-dev`
- **SISL lib**: `git clone https://github.com/SINTEF-Geometry/SISL.git`
- **qt5-libraries**: When building the package on the catamaran you don't need the **ulisse_map** sub-package (so you can add a COLCON_IGNORE file inside it), and these dependencies can be skipped.Otherwise, to use the interface, type:

`sudo apt install libgeographic-* libeigen3-dev libgps-dev python-gps qtquickcontrols2-5-dev qtlocation5-dev qtpositioning5-dev qml-module-qtquick-controls2 qml-module-qtlocation qml-module-qt-labs-* qml-module-qtpositioning qml-module-qtquick-extras qml-module-qtgraphicaleffects qml-module-qtquick-dialogs qml-module-qtquick-controls python3-colcon-common-extensions libconfig++-dev python3-pip libz3-dev`

### Additional packages not coming with ros2

- **colcon**: Follow the guide at https://colcon.readthedocs.io/en/master/user/installation.html
- **rosbag2**: Follow the guide at https://github.com/ros2/rosbag2



## Build

In the following instructions, the `sourceros#` commands are placeholders for your ROS sourcing commands.
If you're using both ROS1 and ROS2 it's important that you define separate aliases in your **~/.bashrc** to source your workspace, that are **not** automatically called when launching new terminals, otherwise you will mix environment variables from different ROS versions.

### ROS2 only

After installing all the needed dependencies, just execute the following commands:

```
#!bash
sourceros2
colcon build --symlink-install
```

### Alongside ROS1

Be sure to start from a clean workspace, with no _log_, _install_ or _build_ folders. First install the needed dependencies, and clone the ros_bridge in your ROS2 workspace (for more info visit the official repo https://github.com/ros2/ros1_bridge):


```
git clone -b <ros2-version> https://github.com/ros2/ros1_bridge.git
```

 Navigate to your ROS1 workspace and build the `ulisse_rosbag_ros1` package (https://bitbucket.org/isme_robotics/ulisse_rosbag_ros1/):

```
#!bash
sourceros1
catkin_make
```

Building the ROS 1 bridge can consume a tremendous amount of memory (almost 4 GB of free RAM per thread while compiling) to the point that it can easily overwhelm a computer if done with parallel compilation enabled.
As such, we recommend first building everything else as usual, then coming back to build the ROS 1 bridge without parallel compilation.

The bridge uses `pkg-config` to find ROS 1 packages.
ROS 2 packages are found through CMake using `find_package()`.
Therefore the `CMAKE_PREFIX_PATH` must not contain paths from ROS 1 which would overlay ROS 2 packages.

You should first build everything but the ROS 1 bridge with normal make arguments.
We don't recommend having your ROS 1 environment sourced during this step as it can add OpenCV 3 to your path.
The ROS 2 image demos you build in this step would then use OpenCV 3 and require it to be on your path when you run them, while the standard installation on Ubuntu Xenial is OpenCV 2.

```
source /opt/ros/crystal/setup.bash
colcon build --symlink-install --packages-skip ros1_bridge
```

Next you need to source the ROS 1 environment, for Linux and ROS Melodic that would be:

```
source /opt/ros/melodic/setup.bash
```

The bridge will be built with support for any message/service packages that are on your path and have an associated mapping between ROS 1 and ROS 2.
Therefore you must add any ROS 1 or ROS 2 workspaces that have message/service packages that you want to be bridged to your path before building the bridge.
This can be done by adding explicit dependencies on the message/service packages to the `package.xml` of the bridge, so that `colcon` will add them to the path before it builds the bridge.
Alternatively you can do it manually by sourcing the relevant workspaces yourself, e.g.:

```
# You have already sourced your ROS installation.
# Source your ROS 2 installation:
. <install-space-with-ros2>/local_setup.bash
# And if you have a ROS 1 overlay workspace, something like:
# . <install-space-to-ros1-overlay-ws>/setup.bash
# And if you have a ROS 2 overlay workspace, something like:
# . <install-space-to-ros2-overlay-ws>/local_setup.bash
```

Then build just the ROS 1 bridge:

```
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
```

*Note:* If you are building on a memory constrained system you might want to limit the number of parallel jobs by setting e.g. the environment variable `MAKEFLAGS=-j1`.


## Usage

First of all you need to export the ROS1 master URI for all your bashes, so it can be convenient to add this line to your **~/.bashrc**: `export ROS_MASTER_URI=http://localhost:11311/`.

### Run the architecture
Firstly be sure to **synchronize your system** time using the GPS, and then configure the GPS itself, by running these commands in the root of the repository:

```
python gpstime.py
ros2 launch ulisse_driver launchGPSSetup.py
```

Wait till the launch files finishes printing info, then you can kill the process if hanging.

The following launch files will run all the necessary nodes (all with `sourceros2`):
```
#!bash
# Shell A (driver)
ros2 launch ulisse_driver launchDriver.py  # real case
-or-
ros2 launch ulisse_sim launchSim.py        # simulating

# Shell B (controller)
ros2 launch ulisse_ctrl launchControl.py

# Shell C (optional, GUI)
ros2 run ulisse_map ulisse_map_node

# Shell D (optional, data monitor)
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

## Available textual consoles

```
ros2 run ulisse_driver driver_console_node

ros2 run ulisse_ctrl controller_console_node
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
