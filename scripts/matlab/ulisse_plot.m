% This is how you can use the import and plot functions

is_sim = 0;

data = import_data("/home/wanderfra/logs/stsm/CSV/rosbag2_2023-03-01_12.11.24/", is_sim);

plot_data(data, is_sim)
