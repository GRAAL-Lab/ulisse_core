function plot_LOS(data, import_sim)
%clc 
%close all

is_sim = 0;

if nargin > 1
    is_sim = import_sim;
end

figure;
title ('LOS Path Following')
subplot(2,1,1); 
h = plot(data.pathfollow.ros_time, data.pathfollow.y, data.pathfollow.ros_time, data.pathfollow.y_real);
xlabel('Time [s]');
ylabel('Cross Track Error [m]');
legend('estimated', 'real');
xlim([0 400]);

subplot(2,1,2)
h = plot(data.pathfollow.ros_time, data.pathfollow.delta);
xlabel('Time [s]');
ylabel('Look-ahead distance \Delta  [m]');
xlim([0 400]);

end

