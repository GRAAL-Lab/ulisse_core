function plot_data(data, import_sim)
clc 
close all

is_sim = 0;

if nargin > 1
    is_sim = import_sim;
end

j = 1;
for i = 1:length(data.gps.cog)
    while (j < (length(data.magnetometer.time)-1) && data.magnetometer.time(j) < data.gps.time(i))
        j = j+1;
    end
    %angleDiff = angdiff(data.filter.state(i,6), data.gps.cog(i)*pi/180);
    angleDiff = angdiff(-atan2(data.magnetometer.xyz(j,2), data.magnetometer.xyz(j,1)), data.gps.cog(i)*pi/180);
    %angleDiff = angdiff(data.compass.rpy(j,3), data.gps.cog(i)*pi/180);
    
    surge(i) =  data.gps.speed(i) * cos(angleDiff);
end

for i = 1:length(data.magnetometer.time)
    data.magnetometer.theta(i) = -atan2(data.magnetometer.xyz(i,2), data.magnetometer.xyz(i,1));
end
b = [0,0.000154653070264672,0.000574020520228603,0.000133110853855716];
a = [1,-2.71451225410788,2.45619225923395,-0.740818220681718];
surgefilt = filter(b,a,surge);

b = [0.01];
a = [1,-0.99];
gyrozfilt = filter(b,a,data.imu.gyroscope(:,3));

figure(2); hold off;
subplot(3,1,1)
h = plot(data.filter.ros_time, data.filter.state(:,4)*180/pi); hold on; set(h,{'DisplayName'},{'roll'})
if (is_sim)
    h = plot(data.sim.ros_time, data.sim.state(:,4)*180/pi, '--'); set(h,{'DisplayName'},{'sim roll'})
end
h = plot(data.compass.ros_time, data.compass.rpy(:,1)*180/pi, '*');  set(h,{'DisplayName'},{'data.compass roll'})
xlabel('Time [s]');
ylabel('Roll angle [deg]');
%ylim([-10,10])
legend('off'); legend('show')
subplot(3,1,2)
h = plot(data.filter.ros_time, data.filter.state(:,5)*180/pi); hold on; set(h,{'DisplayName'},{'pitch'})
if (is_sim)
    h = plot(data.sim.ros_time, data.sim.state(:,5)*180/pi, '--'); set(h,{'DisplayName'},{'sim pitch'})
end
h = plot(data.compass.ros_time, data.compass.rpy(:,2)*180/pi, '*');  set(h,{'DisplayName'},{'data.compass pitch'})

xlabel('Time [s]');
ylabel('Pitch angle [deg]');
legend('off'); legend('show')
subplot(3,1,3)
h = plot(data.filter.ros_time, data.filter.state(:,6)*180/pi); hold on; set(h,{'DisplayName'},{'yaw'})
if (is_sim)
    h = plot(data.sim.ros_time, data.sim.state(:,6)*180/pi, '--'); set(h,{'DisplayName'},{'sim yaw'})
end
h = plot(data.compass.ros_time, data.compass.rpy(:,3)*180/pi, '*');  set(h,{'DisplayName'},{'data.compass yaw'})

h = plot(data.gps.time, data.gps.cog);
h = plot(data.magnetometer.ros_time, data.magnetometer.theta*180/pi);

xlabel('Time [s]');
ylabel('Yaw angle [deg]');
legend('off'); legend('show')

figure(3); hold off;
h = plot(data.filter.ros_time, data.filter.state(:,7:9)); hold on; set(h,{'DisplayName'},{'surge';'sway';'heave'})
if (is_sim == 1)
    h = plot(data.sim.ros_time, data.sim.state(:,7:9), '--');  set(h,{'DisplayName'},{'sim surge';'sim sway';'sim heave'})
end
xlabel('Time [s]');
ylabel('Linear Velocities [m/s]');
legend('off'); legend('show')

figure(4); hold off;
subplot(3,1,1)
h = plot(data.filter.ros_time, data.filter.state(:,10));  hold on; set(h,{'DisplayName'},{'omega_x'})
if (is_sim) 
    h = plot(data.sim.ros_time, data.sim.state(:,10), '--'); set(h,{'DisplayName'},{'sim omega_x'})
end
h = plot(data.imu.ros_time, data.imu.gyroscope(:,1), '*'); set(h,{'DisplayName'},{'gyro_x'})
legend('off'); legend('show')
subplot(3,1,2)
h = plot(data.filter.ros_time, data.filter.state(:,11)); hold on;  set(h,{'DisplayName'},{'omega_y'})
if (is_sim)
    h = plot(data.sim.ros_time, data.sim.state(:,11), '--');   set(h,{'DisplayName'},{'sim_y'})
end 
h = plot(data.imu.ros_time, data.imu.gyroscope(:,2), '*');  set(h,{'DisplayName'},{'gyro_y'})
legend('off'); legend('show')
subplot(3,1,3)
h = plot(data.filter.ros_time, data.filter.state(:,12)); hold on;  set(h,{'DisplayName'},{'omega_z'})
if (is_sim)
    h = plot(data.sim.ros_time, data.sim.state(:,12), '--');   set(h,{'DisplayName'},{'sim_z'})
end
h = plot(data.imu.ros_time, data.imu.gyroscope(:,3), '*');  set(h,{'DisplayName'},{'gyro_z'})
h = plot(data.imu.ros_time, gyrozfilt); 

xlabel('Time [s]');
ylabel('Angular Velocities [rad/s]');
legend('off'); legend('show')

figure(5); hold off;
h = plot(data.filter.ros_time, data.filter.state(:,13:14)); hold on; set(h,{'DisplayName'},{'current_x';'current_y'})
if (is_sim)
    h = plot(data.sim.ros_time, data.sim.state(:,13:14), '--'); set(h,{'DisplayName'},{'sim current_x';'sim current_y'})
end
xlabel('Time [s]');
ylabel('Water Velocity [m/s]');
legend('off'); legend('show')

figure(6); hold off;
h = plot(data.filter.ros_time, data.filter.state(:,15:17)); hold on; set(h,{'DisplayName'},{'bias_x';'bias_y';'bias_z'})
if (is_sim)
    h = plot(data.sim.ros_time, data.sim.state(:,15:17), '--'); set(h,{'DisplayName'},{'sim bias_x';'sim bias_y';'sim bias_z'})
end
xlabel('Time [s]');
ylabel('Gyroscope Bias [rad/s]');
legend('off'); legend('show')

figure(7); hold off;
h = plot(data.filter.ros_time, [data.filter.bodyF_absvel(:,1) data.filter.state(:,12)]); hold on; set(h,{'DisplayName'},{'surge';'omega_z'})
if (is_sim)
    h = plot(data.sim.ros_time, data.sim.state(:,[7 12]), '--'); set(h,{'DisplayName'},{'sim surge';'sim omega_z'})
end
h = plot(data.dcl_reference.ros_time, [data.dcl_reference.desired_surge, data.dcl_reference.desired_yaw_rate], '-.'); set(h,{'DisplayName'},{'desired surge';'desired omega_z'})
xlabel('Time [s]');
ylabel('Absolute Surge and Yawrate [m & rad/s]');
legend('off'); legend('show')

figure(8); hold off;
h = plot(data.thrusters.ros_time, data.thrusters.percentage.*10, '--'); hold on;
h = plot(data.motors.ros_time, data.motors.rpm); hold on;
xlabel('Time [s]');
ylabel('Motors Reference [%*10] and RPMs');
legend('port', 'startboard','port rpm', 'starboard rpm');
%ylim([-100, 100])
%figure(15); hold off;


% figure(9); hold off;
% h = plot(data.filter.state(:,2), data.filter.state(:,1)); hold on; set(h,{'DisplayName'},{'estimated'})
% if (is_sim)
%     h = plot(data.sim.state(:,2), data.sim.state(:,1), '--'); set(h,{'DisplayName'},{'sim'})
% end
% plot(data.gps.longitude, data.gps.latitude, '*'); set(h,{'DisplayName'},{'data.gps'})
% xlabel('Longitude');
% ylabel('Latitude');
% legend('off'); legend('show')


% figure(10); hold off;
% h = plot(data.filter.ned(:,2), data.filter.ned(:,1)); hold on; set(h,{'DisplayName'},{'estimated'})
% if (is_sim)
%     h = plot(data.sim.ned(:,2), data.sim.ned(:,1), '--'); set(h,{'DisplayName'},{'sim'})
% end
% h = plot(data.gps.ned(:,2), data.gps.ned(:,1), '*'); set(h,{'DisplayName'},{'data.gps'})
% axis equal
% xlabel('Easting');
% ylabel('Northing');
% legend('off'); legend('show')

%diff = data.filter.ned(:,1:2) - data.gps.ned(:,1:2);
%figure(11)
%plot(vecnorm(diff,2,2))
%ylabel('Distance norm between estimated and data.gps position [m]');

% figure(12); hold off;
% h = plot(data.filter.ros_time, data.filter.bodyF_cur);
% ylabel('Estimated current projected on body frame [m/s]');

figure(13); %hold off;
h = plot(data.filter.ros_time, data.filter.bodyF_absvel);  hold on;
h = plot(data.gps.ros_time, data.gps.speed);
h = plot(data.gps.ros_time, surgefilt);
ylabel('Estimated absolute velocity projected on body frame [m/s]');

figure(14); %hold off;
h = plot(data.filter.ros_time, data.filter.worldF_absvel); hold on;
ylabel('Estimated absolute velocity projected on world frame [m/s]');


% figure(15); hold off;
% h = plot(data.imu.ros_time, data.imu.accelerometer); hold on;
% xlabel('Time [s]');
% ylabel('Acceleration [m/s^2]');
% legend('x', 'y', 'z');

figure(16); %hold off;
h = plot(data.stsm.ros_time, data.stsm.sigma_surge);  hold on;
h = plot(data.stsm.ros_time, data.stsm.sigma_yawrate);
ylabel('sigma [m/s]');

figure(17); %hold off;
h = plot(data.stsm.ros_time, data.stsm.tau_surge);  hold on;
h = plot(data.stsm.ros_time, data.stsm.tau_yawrate);
ylabel('Control torque [N*m]');


end


