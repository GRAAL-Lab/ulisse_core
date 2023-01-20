# Ulisse Catamaran Control

Ulisse catamaran controller revamped with ROS2.

## Subpackages

Short description of all the packages included in this meta package:

- **surface_vehicle_model**: Dynamic model of a surface vehicle with two thrusters.
- **navigation_filter**: The sensor filtering package providing the vehicle status and seacurrent estimation to the controller.
- **ulisse_msgs**: Interface and services messages package with headers for topic names and common variables.
- **ulisse_ctrl**: The catamaran controller.
- **ulisse_driver**: The low level driver that communicates with the catamaran microcontrollers.
- **ulisse_sim**: The dynamic simulator, which makes use of `surface_vehicle_model` library or replays data from logs.
- **ulisse_map**: Graphical interface (Qt based) for controlling the catamaran.
- **csv_pkg**: Node to log all necessary data to be plotted in matlab.

## Dependencies

In order of installation to respect dependencies:

- **ros2 galactic**: https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html
- **rml**: http://bitbucket.org/isme_robotics/rml
- **fsm**: http://bitbucket.org/isme_robotics/fsm
- **tpik**: http://bitbucket.org/isme_robotics/tpik
- **ikcl**: https://bitbucket.org/isme_robotics/ikcl
- **libgps**: `sudo apt install libgps-dev`
- **SISL lib**: `git clone https://github.com/SINTEF-Geometry/SISL.git`
- **ctrl_toolbox**: http://bitbucket.org/isme_robotics/ctrl_toolbox
- **sisl_toolbox**: http://bitbucket.org/isme_robotics/sisl_toolbox
- **qt5-libraries**: When building the package on the catamaran you don't need the **ulisse_map** sub-package (so you can add a COLCON_IGNORE file inside it), and these dependencies can be skipped.Otherwise, to use the interface, type:

`sudo apt install gpsd-clients qtquickcontrols2-5-dev qtlocation5-dev qtpositioning5-dev qml-module-qtquick-controls2 qml-module-qt-labs-settings qml-module-qt-labs-folderlistmodel qml-module-qtlocation  qml-module-qtpositioning qml-module-qtquick-extras qml-module-qtgraphicaleffects qml-module-qtquick-dialogs qml-module-qtquick-controls python3-colcon-common-extensions qml-module-qtqml-models2`


### Additional packages not coming with ros2

- **colcon**: Follow the guide at https://colcon.readthedocs.io/en/master/user/installation.html

## Build

In ROS2 "galactic", the default DDS (CycloneDDS) presented some undefined behaviour. For this reason the default DDS has been switched to FastRTPS. In addition, in the following instructions, the `sourceros#` commands are placeholders for your ROS sourcing commands. To do so, add this lines in your `~/.bashrc` :

```bash
RMW_IMPLEMENTATION=rmw_fastrtps_cpp; export RMW_IMPLEMENTATION
alias sourceros2='source /opt/ros/galactic/setup.bash && source ~/*ros_ws_folder_name*/install/setup.bash'

```

⚠️ If you're using both ROS1 and ROS2 it's important that you define separate aliases in your `~/.bashrc` to source your workspace, that are **not** automatically called when launching new terminals, otherwise you will mix environment variables from different ROS versions.

To install the dependencies altogether in can use the `install_ulisse_core.sh` script inside the **scripts** folder. After installing all the needed dependencies, to build the repo for the first time:

- Create a ros2 workspace using `colcon` (https://docs.ros.org/en/galactic/Tutorials/Colcon-Tutorial.html)
- Clone in the **src** folder the ulisse_core repo (git clone git@bitbucket.org:isme_robotics/ulisse_core.git)
- Source the workspace with `sourceros2`
- Build ulisse_msgs as first: `colcon build --symlink-install --packages-select ulisse_msgs`
- Then compile all the other pakages: `colcon build --symlink-install`

All the next times, just execute the following commands:

```bash

sourceros2
colcon build --symlink-install
```

## Run the architecture
Firstly be sure to **synchronize your system** time using the GPS, and then configure the GPS itself, by running these commands in the root of the repository:

```bash

python gpstime.py
ros2 launch ulisse_driver launchGPSSetup.py
```

Wait till the launch files finishes printing info, then you can kill the process if hanging.

The following launch files will run all the necessary nodes (all with `sourceros2`):

```bash

# Shell A (driver)
ros2 launch ulisse_driver launchDriver.py  # real case
-or-
ros2 launch ulisse_sim launchSim.py        # simulating

# Shell B (controller)
ros2 launch ulisse_ctrl launchControl.py

# Shell C (optional, GUI)
ros2 run ulisse_map ulisse_map_node
```

## GPS Setup
To enable the GPS on a fresh install of Linux you will have to:

- Add in **/etc/default/gpsd** the following line: `DEVICE:"/dev/ttyS1"`.
- Add your user to the `dialout` user group.


### Testing the serial

⚠️ Be sure that in the driver configuration file *ulisse_driver/conf/ulisse_driver.conf* the **SerialDevice** paramater is set to `"/tmp/serial1"`. Then, run the following commands in three separate ROS2 sourced terminals (`sourceros2` command):


```bash

# Shell A (setup serial)
socat -d -d pty,raw,echo=0,link=/tmp/serial1 pty,raw,echo=0,link=/tmp/serial2

# Shell B (launch driver)
ros2 launch ulisse_driver launchDriver.py

# Shell C (echo on any topic of interest, e,g. /sensor/ambient)
ros2 topic echo /sensor/ambient

# Shell D (transmit data on serial)
# '80' is the limit of bytes/sec, to slow down the cat command output
cat serial_file.log | pv -l -L 80 -q > /tmp/serial2
```

You can also use the file *play_serial.sh* to continuosly replay the logfile.


## Configure Ethernet-WiFi Bridge

### Windows (host) side:

- Connect to WiFi Hotspot.
- Connect ethernet cable to catamaran.
- Share WiFi connection with "Ethernet" in Control Panel: Network Connections->*Left Click on* WiFi Connection->Properties->Sharing Tab.
- Set the IPv4 of the Ethernet Connection as 192.168.1.169 (and it's gateway as the IP of the WiFi connection -> UPDATE: leave the gateway blank or it doesn't work).

### Ubuntu side (Catamaran or VirtualBox):

- Use the following command to set the correct gateway:
sudo route add default gw 192.168.1.169


## Misc

For additional info look [info.txt](./info.txt).
