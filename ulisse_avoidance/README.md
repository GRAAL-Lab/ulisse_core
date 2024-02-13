# Ulisse Avoidance

ROS2-based interface between Ulisse Core and the [Obstacle Avoidance Library](https://bitbucket.org/isme_robotics/oal).

## Build and Run
The package depends on the [Obstacle Avoidance Library](https://bitbucket.org/isme_robotics/oal): build it as c++ package. 
Also install the [Obstacle Publisher](https://bitbucket.org/isme_robotics/ros2_obstacle_publisher) in your ros2_ws in order to test this.  
Run with `ros2 run ulisse_avoidance oal_interface` and send commands through the GUI after checking the **Avoidance Planner** box.
Also check the **COLREGS** box for COLREGs-compliant plans.

## Author
Samuele Depalo (thesist)  
depalo.samuele@gmail.com