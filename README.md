# Ulisse Catamaran Control for ASV/ROV cooperative system

Ulisse catamaran controller revamped with ROS2.

## Subpackages

Short description of all the packages included in this meta package:

- **surface_vehicle_model**: Dynamic model of a surface vehicle with two thrusters.
- **navigation_filter**: The sensor filtering package providing the vehicle status and sea current estimation to the controller.
- **ulisse_msgs**: Interface and services messages package with headers for topic names and common variables.
- **ulisse_ctrl**: The catamaran controller.
- **ulisse_driver**: The low level driver that communicates with the catamaran micro-controllers.
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

⚠️ If you're using both ROS1 and ROS2 it's important that you define separate aliases in your `~/.bashrc` to source your workspace, that are **not** automatically called when launching new terminals, otherwise you will mix environment variables from different ROS versions.

To install the dependencies altogether in can use the `install_ulisse_core.sh` script inside the **scripts** folder. After installing all the needed dependencies, to build the repo for the first time:

- Create a ros2 workspace using `colcon` (https://docs.ros.org/en/galactic/Tutorials/Colcon-Tutorial.html)
- Clone in the **src** folder the ulisse_core repo (git clone git@bitbucket.org:isme_robotics/ulisse_core.git)
- Source the workspace with `sourceros2`
- Then compile all the other pakages: `colcon build --symlink-install`

All the next times, just execute the following commands:

```bash

sourceros2
colcon build --symlink-install
```

## Run the ULISSE architecture


```bash

ros2 launch ulisse_sim launchSim.py        # simulating

ros2 launch ulisse_ctrl launchControl.py   # control

ros2 run ulisse_ctrl controller_console_node # user interface
```

## Run the BlueROV2 architecture


```bash

ros2 launch rov_ctrl launchControl.py      # control

ros2 run rov_sim simulator_node		   # simulating

ros2 run rov_ctrl controller_console_node # user interface
```

## Run for Visualization

To launch the **Rviz**, type on a separate shell of your on-shore device type (no need for screen here):

```bash
rviz2
```

