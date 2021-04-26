ros2 launch csv_pkg launchExperiment.py

ros2 run csv_pkg stop_experiments 

python finish_experiment.py "Nome Esperimento"

ros2 run csv_pkg make_circle 10

ros2 run csv_pkg make_ellipse 20 10

####### TERMINAL #######

ssh graal@192.168.1.100

sourceros2

ros2 launch ulisse_sim launchSim.py
---- or ----
ros2 launch ulisse_driver launchDriver.py

ros2 run ulisse_ctrl monitor_node

ros2 launch ulisse_ctrl launchControl.py

ros2 run ulisse_map ulisse_map_node