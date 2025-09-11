function plot_ILOS(data, import_sim)
%clc 
%close all

is_sim = 0;

if nargin > 1
    is_sim = import_sim;
end

figure;
title ('ILOS Path Following')
subplot(4,1,1)
h = plot(data.pathfollow.ros_time, data.pathfollow.y, data.pathfollow.ros_time, data.pathfollow.y_real);
xlabel('Time [s]');
ylabel('Cross Track Error [m]');
legend('estimated', 'real');
xlim([0 400]);

subplot(4,1,2)
h = plot(data.pathfollow.ros_time, data.pathfollow.delta);
xlabel('Time [s]');
ylabel('Look-ahead distance \Delta [m]');
xlim([0 400]);

subplot(4,1,3)
h = plot(data.pathfollow.ros_time, data.pathfollow.y_int, ...
    data.pathfollow.ros_time, data.pathfollow.y_int_dot);
xlabel('Time [s]');
ylabel('Integral part');
legend('y_{int}', '\dot{y_{int}}');
xlim([0 400]);

subplot(4,1,4)
h = plot(data.pathfollow.ros_time, data.pathfollow.psi*180/pi);
xlabel('Time [s]');
ylabel('\psi_{ilos} [degree]');
xlim([0 400]);
%figure (2)

end

