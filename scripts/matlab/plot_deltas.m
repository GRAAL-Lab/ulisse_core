function plot_deltas(data1, data2, import_sim)
%clc 
%close all

is_sim = 0;

if nargin > 1
    is_sim = import_sim;
end

%figure;
%subplot(2,1,1)
h = plot(data1.pathfollow.ros_time, data1.pathfollow.y, ...
    data2.pathfollow.ros_time, data2.pathfollow.y);
export_coolfig('-pdf','yes',h,'ILOS','time [s]','Cross Track Error [m]');
%xlabel('Time [s]');
%ylabel('Cross Track error [m]');
%xlim([0 400]);
%legend('\Delta=variable', '\Delta=fixed');

%subplot(2,1,2)
%h = plot(data1.pathfollow.ros_time, data1.pathfollow.delta, ...
%    data2.pathfollow.ros_time, data2.pathfollow.delta);
%xlabel('Time [s]');
%ylabel('Look-ahead distance \Delta [m]');
%xlim([0 400]);
%legend('y_{int}', '\dot{y_{int}}');

end

