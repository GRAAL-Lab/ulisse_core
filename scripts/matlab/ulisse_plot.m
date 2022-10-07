% This is how you can use the import and plot functions
clc
close all

is_sim = 0;
% Import data of multiple Lines
% data = import_data("/home/wanderfra/logs/csv/sensors_calib_jan22/rosbag2_2022-01-12_12.25.50/", is_sim);
data_multiLine_ILOS_delta = import_data("/home/graal/Desktop/juri-thesis/simulation/csv/rosbag2_2022-10-06_11.47.48/", is_sim);
data_multiLine_ILOS = import_data("/home/graal/Desktop/juri-thesis/simulation/csv/rosbag2_2022-10-06_16.49.04/", is_sim);

data_multiLine_LOS_delta = import_data("/home/graal/Desktop/juri-thesis/simulation/csv/rosbag2_2022-10-06_11.17.00/", is_sim);
data_multiLine_LOS = import_data("/home/graal/Desktop/juri-thesis/simulation/csv/rosbag2_2022-10-06_11.37.09/", is_sim);

data_multiLine_CurrrentEst_delta = import_data("/home/graal/Desktop/juri-thesis/simulation/csv/rosbag2_2022-10-06_14.45.27/",is_sim);
data_multiLine_CurrrentEst = import_data("/home/graal/Desktop/juri-thesis/simulation/csv/rosbag2_2022-10-06_14.56.22/", is_sim);
%% Multiple Lines (How each algorithm works)
% ILOS
h = figure(1);
data1 = data_multiLine_ILOS_delta;
plot(data1.pathfollow.ros_time, data1.pathfollow.y, ...
    data1.pathfollow.ros_time, data1.pathfollow.y_real);
xlim([0 400]);
%legend({'\Delta=variable', '\Delta=fixed'},'Location','southeast');
grid minor
legend('estimated', 'real');
export_coolfig('Multiple_Algorithm_y_ILOS.pdf','-pdf',h,'','time [s]','Cross Track Error [m]');
%%
h = figure(1);
data1 = data_multiLine_ILOS_delta;
plot(data1.pathfollow.ros_time, data1.pathfollow.delta);
xlim([0 400]);
%legend({'\Delta=variable', '\Delta=fixed'},'Location','southeast');
grid minor
%legend('estimated', 'real');
export_coolfig('Multiple_Algorithm_delta_ILOS.pdf','-pdf',h,'','time [s]','Look-ahead distance \Delta [m]');

%%
h = figure(1);
data1 = data_multiLine_ILOS_delta;
plot(data1.pathfollow.ros_time, data1.pathfollow.y_int, ...
    data1.pathfollow.ros_time, data1.pathfollow.y_int_dot);
xlim([0 400]);
%legend({'\Delta=variable', '\Delta=fixed'},'Location','southeast');
grid minor
legend({'y_{int}', '\dot{y_{int}}'},'Location','southeast');
%legend('estimated', 'real');
export_coolfig('Multiple_Algorithm_integral_ILOS.pdf','-pdf',h,'','time [s]','Integral terms');

%%
h = figure(1);
data1 = data_multiLine_ILOS_delta;
plot(data1.pathfollow.ros_time, data1.pathfollow.psi*180/pi);
xlim([0 400]);
%legend({'\Delta=variable', '\Delta=fixed'},'Location','southeast');
grid minor
%legend('estimated', 'real');
export_coolfig('Multiple_Algorithm_psi_ILOS.pdf','-pdf',h,'','time [s]','\psi_{ilos} [degree]');

%% LOS
h = figure(2);
data1 = data_multiLine_LOS_delta;
plot(data1.pathfollow.ros_time, data1.pathfollow.y, ...
    data1.pathfollow.ros_time, data1.pathfollow.y_real);
xlim([0 400]);
%legend({'\Delta=variable', '\Delta=fixed'},'Location','southeast');
grid minor
legend('estimated', 'real');
export_coolfig('Multiple_Algorithm_y_LOS.pdf','-pdf',h,'','time [s]','Cross Track Error [m]');
%%
h = figure(1);
data1 = data_multiLine_LOS_delta;
plot(data1.pathfollow.ros_time, data1.pathfollow.delta);
xlim([0 400]);
%legend({'\Delta=variable', '\Delta=fixed'},'Location','southeast');
grid minor
%legend('estimated', 'real');
export_coolfig('Multiple_Algorithm_delta_LOS.pdf','-pdf',h,'','time [s]','Look-ahead distance \Delta [m]');
%% Curretn Est
h = figure(3);
data1 = data_multiLine_CurrrentEst_delta;
plot(data1.pathfollow.ros_time, data1.pathfollow.y, ...
    data1.pathfollow.ros_time, data1.pathfollow.y_real);
xlim([0 310]);
%legend({'\Delta=variable', '\Delta=fixed'},'Location','southeast');
grid minor
legend('estimated', 'real');
export_coolfig('Multiple_Algorithm_y_Current.pdf','-pdf',h,'','time [s]','Cross Track Error [m]');
%%
h = figure(1);
data1 = data_multiLine_CurrrentEst_delta;
plot(data1.pathfollow.ros_time, data1.pathfollow.delta);
xlim([0 310]);
%legend({'\Delta=variable', '\Delta=fixed'},'Location','southeast');
grid minor
%legend('estimated', 'real');
export_coolfig('Multiple_Algorithm_delta_CurrentEst.pdf','-pdf',h,'','time [s]','Look-ahead distance \Delta [m]');
%% Multiple Line
h = figure(1)
data1 = data_multiLine_ILOS_delta;
data2 = data_multiLine_ILOS;
plot(data1.pathfollow.ros_time, data1.pathfollow.y, ...
    data2.pathfollow.ros_time, data2.pathfollow.y);
xlim([0 400]);
legend('\Delta=variable', '\Delta=fixed');
%legend({'\Delta=variable', '\Delta=fixed'},'Location','southeast');
grid minor
export_coolfig('Multiple_ILOS.pdf','-pdf',h,'','time [s]','Cross Track Error [m]');

rms(data2.pathfollow.y)
%% LOS
h = figure(2)
data1 = data_multiLine_LOS_delta;
data2 = data_multiLine_LOS;
plot(data1.pathfollow.ros_time, data1.pathfollow.y, ...
    data2.pathfollow.ros_time, data2.pathfollow.y);
xlim([0 400]);
legend('\Delta=variable', '\Delta=fixed');
%legend({'\Delta=variable', '\Delta=fixed'},'Location','southeast');
grid minor
export_coolfig('Multiple_LOS.pdf','-pdf',h,'','time [s]','Cross Track Error [m]');

rms(data1.pathfollow.y)
%% CurrentEst
h = figure(3)
data1 = data_multiLine_CurrrentEst_delta;
data2 = data_multiLine_CurrrentEst;
plot(data1.pathfollow.ros_time, data1.pathfollow.y, ...
    data2.pathfollow.ros_time, data2.pathfollow.y);
xlim([0 300]);
legend('\Delta=variable', '\Delta=fixed');
%legend({'\Delta=variable', '\Delta=fixed'},'Location','southeast');
grid minor
export_coolfig('Multiple_CurrentEst.pdf','-pdf',h,'','time [s]','Cross Track Error [m]');

rms(data1.pathfollow.y)
%% Multiple Lines d10
% Import data of multiple Lines
data_multiLineD10_ILOS_delta = import_data("/home/graal/Desktop/juri-thesis/simulation/csv/rosbag2_2022-10-06_17.51.31/", is_sim);
data_multiLineD10_ILOS = import_data("/home/graal/Desktop/juri-thesis/simulation/csv/rosbag2_2022-10-06_17.37.11/", is_sim);

data_multiLineD10_LOS_delta = import_data("/home/graal/Desktop/juri-thesis/simulation/csv/rosbag2_2022-10-07_10.08.17/", is_sim);
data_multiLineD10_LOS = import_data("/home/graal/Desktop/juri-thesis/simulation/csv/rosbag2_2022-10-07_10.24.00/", is_sim);

data_multiLineD10_CurrrentEst_delta = import_data("/home/graal/Desktop/juri-thesis/simulation/csv/rosbag2_2022-10-07_10.50.48/",is_sim);
data_multiLineD10_CurrrentEst = import_data("/home/graal/Desktop/juri-thesis/simulation/csv/rosbag2_2022-10-07_10.38.05/", is_sim);
%% Multiple Lines d10
% Draw plot
% ILOS
h = figure(1)
data1 = data_multiLineD10_ILOS_delta;
data2 = data_multiLineD10_ILOS;
plot(data1.pathfollow.ros_time, data1.pathfollow.y, ...
    data2.pathfollow.ros_time, data2.pathfollow.y);
xlim([0 800]);
legend('\Delta=variable', '\Delta=fixed');
%legend({'\Delta=variable', '\Delta=fixed'},'Location','southeast');
grid minor
export_coolfig('MultipleD10_ILOS.pdf','-pdf',h,'','time [s]','Cross Track Error [m]');

rms(data1.pathfollow.y)
%% LOS

h = figure(2)
data1 = data_multiLineD10_LOS_delta;
data2 = data_multiLineD10_LOS;
plot(data1.pathfollow.ros_time, data1.pathfollow.y, ...
    data2.pathfollow.ros_time, data2.pathfollow.y);
xlim([0 800]);
legend('\Delta=variable', '\Delta=fixed');
%legend({'\Delta=variable', '\Delta=fixed'},'Location','southeast');
grid minor
export_coolfig('MultipleD10_LOS.pdf','-pdf',h,'','time [s]','Cross Track Error [m]');

rms(data1.pathfollow.y)
%% CurrentEst
h = figure(3)
data1 = data_multiLineD10_CurrrentEst_delta;
data2 = data_multiLineD10_CurrrentEst;
plot(data1.pathfollow.ros_time, data1.pathfollow.y, ...
    data2.pathfollow.ros_time, data2.pathfollow.y);
xlim([0 700]);
legend('\Delta=variable', '\Delta=fixed');
%legend({'\Delta=variable', '\Delta=fixed'},'Location','southeast');
grid minor
export_coolfig('MultipleD10_CurrentEst.pdf','-pdf',h,'','time [s]','Cross Track Error [m]');

rms(data1.pathfollow.y)
%% Multiple Lines 90
% Import data
data_multiLine90_ILOS_delta = import_data("/home/graal/Desktop/juri-thesis/simulation/csv/rosbag2_2022-10-07_11.58.43/", is_sim);
data_multiLine90_ILOS = import_data("/home/graal/Desktop/juri-thesis/simulation/csv/rosbag2_2022-10-07_12.10.17/", is_sim);

data_multiLine90_LOS_delta = import_data("/home/graal/Desktop/juri-thesis/simulation/csv/rosbag2_2022-10-07_12.46.09/", is_sim);
data_multiLine90_LOS = import_data("/home/graal/Desktop/juri-thesis/simulation/csv/rosbag2_2022-10-07_12.36.37/", is_sim);

data_multiLine90_CurrrentEst_delta = import_data("/home/graal/Desktop/juri-thesis/simulation/csv/rosbag2_2022-10-07_11.16.33/",is_sim);
data_multiLine90_CurrrentEst = import_data("/home/graal/Desktop/juri-thesis/simulation/csv/rosbag2_2022-10-07_11.27.41/", is_sim);
%% Multiple Lines 90
% ILOS
h = figure(1)
data1 = data_multiLine90_ILOS_delta;
data2 = data_multiLine90_ILOS;
plot(data1.pathfollow.ros_time, data1.pathfollow.y, ...
    data2.pathfollow.ros_time, data2.pathfollow.y);
xlim([0 500]);
legend({'\Delta=variable', '\Delta=fixed'},'Location','southeast');
grid minor
export_coolfig('Multiple90_ILOS.pdf','-pdf',h,'','time [s]','Cross Track Error [m]');

rms(data1.pathfollow.y)
%% LOS
h = figure(2)
data1 = data_multiLine90_LOS_delta;
data2 = data_multiLine90_LOS;
plot(data1.pathfollow.ros_time, data1.pathfollow.y, ...
    data2.pathfollow.ros_time, data2.pathfollow.y);
xlim([0 500]);
legend({'\Delta=variable', '\Delta=fixed'},'Location','southeast');
grid minor
export_coolfig('Multiple90_LOS.pdf','-pdf',h,'','time [s]','Cross Track Error [m]');

rms(data1.pathfollow.y)

%% Current Est
h = figure(3)
data1 = data_multiLine90_CurrrentEst_delta;
data2 = data_multiLine90_CurrrentEst;
plot(data1.pathfollow.ros_time, data1.pathfollow.y, ...
    data2.pathfollow.ros_time, data2.pathfollow.y);
xlim([0 440]);
legend({'\Delta=variable', '\Delta=fixed'},'Location','southeast');
grid minor
export_coolfig('Multiple90_CurrentEst.pdf','-pdf',h,'','time [s]','Cross Track Error [m]');

rms(data1.pathfollow.y)

%% Multiple60_d10 Lines
% Import data
data_multiLine60d10_ILOS_delta = import_data("/home/graal/Desktop/juri-thesis/simulation/csv/rosbag2_2022-10-07_13.47.57/", is_sim);
data_multiLine60d10_ILOS = import_data("/home/graal/Desktop/juri-thesis/simulation/csv/rosbag2_2022-10-07_13.39.45/", is_sim);

data_multiLine60d10_LOS_delta = import_data("/home/graal/Desktop/juri-thesis/simulation/csv/rosbag2_2022-10-07_13.20.49/", is_sim);
data_multiLine60d10_LOS = import_data("/home/graal/Desktop/juri-thesis/simulation/csv/rosbag2_2022-10-07_13.32.05/", is_sim);

data_multiLine60d10_CurrrentEst_delta = import_data("/home/graal/Desktop/juri-thesis/simulation/csv/rosbag2_2022-10-07_11.16.33/",is_sim);
data_multiLine60d10_CurrrentEst = import_data("/home/graal/Desktop/juri-thesis/simulation/csv/rosbag2_2022-10-07_14.11.10/", is_sim);
%% Multiple60_d10 Lines
% ILOS
h = figure(1)
data1 = data_multiLine60d10_ILOS_delta;
data2 = data_multiLine60d10_ILOS;
plot(data1.pathfollow.ros_time, data1.pathfollow.y, ...
    data2.pathfollow.ros_time, data2.pathfollow.y);
xlim([0 450]);
legend({'\Delta=variable', '\Delta=fixed'},'Location','southeast');
grid minor
export_coolfig('Multiple60_d10_ILOS.pdf','-pdf',h,'','time [s]','Cross Track Error [m]');

rms(data1.pathfollow.y)
%% LOS
h = figure(2)
data1 = data_multiLine60d10_LOS_delta;
data2 = data_multiLine60d10_LOS;
plot(data1.pathfollow.ros_time, data1.pathfollow.y, ...
    data2.pathfollow.ros_time, data2.pathfollow.y);
xlim([0 450]);
legend('\Delta=variable', '\Delta=fixed');
%legend({'\Delta=variable', '\Delta=fixed'},'Location','southeast');
grid minor
export_coolfig('Multiple60_d10_LOS.pdf','-pdf',h,'','time [s]','Cross Track Error [m]');

rms(data1.pathfollow.y)

%% CurrentEst
h = figure(3)
data1 = data_multiLine60d10_CurrrentEst_delta;
data2 = data_multiLine60d10_CurrrentEst;
plot(data1.pathfollow.ros_time, data1.pathfollow.y, ...
    data2.pathfollow.ros_time, data2.pathfollow.y);
xlim([0 440]);
legend({'\Delta=variable', '\Delta=fixed'},'Location','southeast');
grid minor
export_coolfig('Multiple60_d10_CurrentEst.pdf','-pdf',h,'','time [s]','Cross Track Error [m]');


rms(data1.pathfollow.y)
