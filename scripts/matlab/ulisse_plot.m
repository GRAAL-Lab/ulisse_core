% This is how you can use the import and plot functions

is_sim = 0;

data = import_data("/home/wanderfra/logs/csv/sensors_calib_jan22/rosbag2_2022-01-12_12.25.50/", is_sim);

plot_data(data, is_sim)