``` 
██╗   ██╗██╗     ██╗███████╗███████╗███████╗
██║   ██║██║     ██║██╔════╝██╔════╝██╔════╝
██║   ██║██║     ██║███████╗███████╗█████╗  
██║   ██║██║     ██║╚════██║╚════██║██╔══╝  
╚██████╔╝███████╗██║███████║███████║███████╗ v2.0
 ╚═════╝ ╚══════╝╚═╝╚══════╝╚══════╝╚══════╝
(graal@192.168.1.100)
```


# Ulisse Catamaran Control

Ulisse catamaran controller revamped with ROS2 (Kilted Kaiju), on Ubuntu 24.04.

## Subpackages

Short description of all the packages included in this meta package:

- **nav_filter**: The sensor filtering package providing the vehicle status and sea current estimation to the controller.
- **ulisse_msgs**: Interface and services messages package with headers for topic names and common variables.
- **ulisse_ctrl**: The catamaran controller.
- **ulisse_driver**: The low level driver that communicates with the catamaran micro-controllers.
- **ulisse_sim**: The dynamic simulator, which makes use of `surface_vehicle_model` library or replays data from logs.
- **ulisse_map**: Graphical interface (Qt based) for controlling the catamaran.
- **csv_pkg**: Node to log all necessary data to be plotted in matlab.
- **bag_recorder**: Programmatic ROSBag recorder, configurable and callable via ROS Service.

## Dependencies

All the dependencies can be installed using the script contained in this repo: [`install_ulisse_core.sh`](https://bitbucket.org/isme_robotics/ulisse_core/src/master/scripts/install_ulisse_core.sh) (see the _Build_ section). The complete list can be found below.

In order of installation to respect dependencies:

- **ros2 kilted**: https://docs.ros.org/en/kilted/Installation/Ubuntu-Install-Debians.html
- **rml**: http://bitbucket.org/isme_robotics/rml
- **fsm**: http://bitbucket.org/isme_robotics/fsm
- **tpik**: http://bitbucket.org/isme_robotics/tpik
- **ikcl**: https://bitbucket.org/isme_robotics/ikcl
- **marine_vehicle_models**: https://bitbucket.org/isme_robotics/marine_vehicle_models
- **libgps**: `sudo apt install libgps-dev`
- **orientus-ros2-driver**: https://bitbucket.org/isme_robotics/orientus-ros2-driver
- **SISL lib**: `git clone https://github.com/SINTEF-Geometry/SISL.git`
- **ctrl_toolbox**: http://bitbucket.org/isme_robotics/ctrl_toolbox
- **sisl_toolbox**: http://bitbucket.org/isme_robotics/sisl_toolbox
- **qt5-libraries**: Installed with the `graal_utils` [script](https://bitbucket.org/isme_robotics/graal_utils/src/master/scripts/install_update_graal_libs.sh). When building the package on the catamaran you don't need the **ulisse_map** sub-package (so you can add a COLCON_IGNORE file inside it), and these dependencies can be skipped.


### Additional packages (for building)

- **colcon**: Follow the guide at https://colcon.readthedocs.io/en/master/user/installation.html

## Build

<!--⚠️ If you're using both ROS1 and ROS2 it's important that you define separate aliases in your `~/.bashrc` to source your workspace, that are **not** automatically called when launching new terminals, otherwise you will mix environment variables from different ROS versions.-->

To install the dependencies altogether you can use the [`install_ulisse_core.sh`](https://bitbucket.org/isme_robotics/ulisse_core/src/master/scripts/install_ulisse_core.sh) script inside the **scripts** folder. After installing all the needed dependencies, to build the repo for the first time:

- Create a ros2 workspace using `colcon` (https://docs.ros.org/en/galactic/Tutorials/Colcon-Tutorial.html)
- Clone in the **src** folder the ulisse_core repo (git clone git@bitbucket.org:isme_robotics/ulisse_core.git)
- Source the workspace with `sourceros2`
- Then compile all the other pakages: `colcon build --symlink-install`

All the next times, just execute the following commands:

```bash

sourceros2
colcon build --symlink-install
```

### Faster Builds on Embedded Systems

For initial builds on the catamaran PC, enable unity builds (30-40% faster, uses more RAM):

```bash
colcon build --symlink-install --cmake-args -DENABLE_UNITY_BUILD=ON --parallel-workers 4
```

For incremental development, disable unity:

```bash
colcon build --symlink-install
```

## Run the architecture

With your on-shore device connect to the Wi-Fi network of the catamaran "ISME AUV", and give yourself a static IP in the network `192.168.1.*`. Then connect via SSH with the catamaran (user:graal, pwd: graal) using:

```bash

ssh graal@192.168.1.100
```

When using remote terminals, to prevent problems related to network failures, it is strongly advised to use `screen` sessions in the terminal where you launch the controller and the driver. Useful commands:

 - Create named session: `screen -S session_name`
 - Detach session: `Ctrl+A`, then press `D`
 - Reattach to session: `screen -r session_name`
 - List all the sessions: `screen -ls`

Once on the catamaran, firstly be sure to configure the GPS itself, and then **synchronize your system time** using the GPS by running these commands:

```bash

ros2 launch ulisse_driver launchGPSSetup.py
cd ros2_ws/src/ulisse_core/scripts
python3 gpstime.py

```

Wait till the launch files finishes printing info, then you can kill the process if hanging.

The following launch files will run all the necessary nodes (all with `sourceros2`):

```bash

# Shell A (driver)
screen -S driver
ros2 launch ulisse_driver launchDriver.py  # real case
-or-
ros2 launch ulisse_sim launchSim.py        # simulating

# Shell B (controller)
screen -S control
ros2 launch ulisse_ctrl launchControl.py
```

## Run the GUI

To launch the **GUI**, type on a separate shell of your on-shore device type (no need for screen here):

```bash
# Shell C (optional, GUI)
ros2 run ulisse_map ulisse_map_node
```

## GPS Setup
To enable the GPS on a fresh install of Linux you will have to:

- Add in **/etc/default/gpsd** the following line: `DEVICE:"/dev/ttyS1"`.
- Add your user to the `dialout` user group: `sudo usermod -a -G dialout graal`..


## Orientus IMU Calibration

To calibrate the Orientus IMU remotely we need to login in the Ulisse PC via ssh, and redirect the USB Serial to a network port (16719):

```
socat -d -d tcp-l:16719,reuseaddr,fork file:/dev/ttyUSB0,b115200,raw
```

Then we can open the Orientus Manager on our remote PC and select "**Network**" in the Communication dropdown menu, then the catamaran **IP** and the port **16719**.


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

## Network Configuration

IP Table:

| Device                  | IP            | hostname      |
|-------------------------|---------------|---------------|
| Main CPU                | 192.168.1.100 | ulisse        |
| Ulisse Wireless Antenna | 192.168.1.251 |               |
| Ground Wireless Antenna | 192.168.1.252 |               |


### Configure Ethernet-WiFi Bridge

#### Windows (host) side:

- Connect to WiFi Hotspot.
- Connect ethernet cable to catamaran.
- Share WiFi connection with "Ethernet" in Control Panel: Network Connections->*Left Click on* WiFi Connection->Properties->Sharing Tab.
- Set the IPv4 of the Ethernet Connection as 192.168.1.169 (and it's gateway as the IP of the WiFi connection -> UPDATE: leave the gateway blank or it doesn't work).

#### Ubuntu side (Catamaran or VirtualBox):

- Use the following command to set the correct gateway:
sudo route add default gw 192.168.1.169



## Miscellaneous

### Select network connection from Command Line

For local `192.168.1.100` network (in deployment stage):

`sudo nmcli connection up ULISSE`

or, for GRAAL `130.251.6.10` network (in development):

`sudo nmcli connection up ULISSE-GRAAL`


### Disable GPS in the Navigation Filter

For some testing of nav_filter and experiments, an optional switch has been added to toggle the use of the GPS by the navigation filter. To enable or disable the use of the GPS just publish a boolean to the topic "`/ulisse/USE_GPS`". To do it via command line:

`ros2 topic pub /ulisse/USE_GPS -1 std_msgs/msg/Bool "{data: false}"`

### PC104 Rebooting issues

If for some reasons, the PC104 is not correctly shutting down or rebooting, using the watchdog solves the issue.

1. Install the watchdog package:

   ```bash
   sudo apt install watchdog
   ```

2. Edit `/etc/systemd/system.conf` and set:

   ```
   RuntimeWatchdogSec=10s
   RebootWatchdogSec=10s
   ```

   * `RuntimeWatchdogSec` = keeps the watchdog fed while system runs.
   * `RebootWatchdogSec` = when reboot is issued, systemd arms the watchdog and *stops feeding it*, so hardware resets in 5s if kernel reset fails.

3. Reload systemd and reboot:

   ```bash
   sudo systemctl daemon-reexec
   sudo reboot
   ```

### Misc

Current compile time, Ubuntu 24.04 on PC104 PCM-3365 (Intel Atom E3845 1.91 GHz), using `colcon build --symlink-install --executor sequential`:

```
Starting >>> adnav_interfaces
Finished <<< adnav_interfaces [2min 36s]                               
Starting >>> surface_vehicle_model
Finished <<< surface_vehicle_model [22.5s]                             
Starting >>> ulisse_msgs
Finished <<< ulisse_msgs [6min 6s]                                
Starting >>> adnav_driver
Finished <<< adnav_driver [2min 36s]                                
Starting >>> bag_recorder
Finished <<< bag_recorder [1min 0s]                             
Starting >>> bags_to_csv
Finished <<< bags_to_csv [3min 15s]                                
Starting >>> ulisse_driver
Finished <<< ulisse_driver [2min 43s]                                
Starting >>> ulisse_sim
Finished <<< ulisse_sim [3min 48s]                                
Starting >>> adnav_launch
Finished <<< adnav_launch [5.40s]                          
Starting >>> nav_filter      
Finished <<< nav_filter [5min 46s]                                
Starting >>> ulisse_ctrl
Finished <<< ulisse_ctrl [13min 27s]                                 

Summary: 11 packages finished [41min 47s]
```

Using `colcon build --symlink-install`:



For additional info look [info.txt](./info.txt).
