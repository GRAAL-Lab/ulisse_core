function plot_CurrentEst(data, import_sim)
%clc 
%close all

is_sim = 0;

if nargin > 1
    is_sim = import_sim;
end

figure;
title ('Current-based Path Following')
subplot(3,1,1)
h = plot(data.pathfollow.ros_time, data.pathfollow.y, data.pathfollow.ros_time, data.pathfollow.y_real);
xlabel('Time [s]');
ylabel('Cross Track Error [m]');
legend('estimated', 'real');
xlim([0 300]);

subplot(3,1,2)
h = plot(data.pathfollow.ros_time, data.pathfollow.delta);
xlabel('Time [s]');
ylabel('Look-ahead distance \Delta [m]');
xlim([0 300]);

subplot(3,1,3)
h = plot(data.filter.ros_time, data.filter.bodyF_cur);
xlabel('Time [s]');
ylabel('Estimated current velocity [m/s]');
legend('x', 'y','z');
xlim([0 300]);

%figure (2)

end

