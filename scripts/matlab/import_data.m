function [data] = import_data(path, import_sim)

is_sim = 0;

if nargin > 1
    is_sim = import_sim;
end


delimiterIn = ',';
headerlinesIn = 1;
first = 10;

filename = path + "nav_filter.txt";
A = importdata(filename, delimiterIn, headerlinesIn);
time = A.data(first:end, 1);

i=1;
data.filter.ros_time = A.data(first:end,i)  - time(1);
i=2;
data.filter.time = A.data(first:end,i)  - time(1);
data.filter.state = A.data(first:end,i+1:i+17);
data.filter.ned = zeros(length(data.filter.state(:,1)),3);
for k=1:length(data.filter.state(:,1))
    [x, y, zone, isnorth] = utmups_fwd(data.filter.state(k,1), data.filter.state(k,2));
    data.filter.ned(k,1) = y;
    data.filter.ned(k,2) = x;
    data.filter.ned(k,3) = data.filter.state(k,3);
    
    data.filter.bodyF_cur(k,:) = rotation(data.filter.state(k,4), data.filter.state(k,5), data.filter.state(k,6))'*[data.filter.state(k,13:14)';0];
    data.filter.bodyF_absvel(k,:) = data.filter.state(k,7:9) + data.filter.bodyF_cur(k,:);
    data.filter.worldF_absvel(k,:) = rotation(data.filter.state(k,4), data.filter.state(k,5), data.filter.state(k,6))*data.filter.bodyF_absvel(k,:)';
    
    if (data.filter.state(k,6) < 0)
        data.filter.state(k,6) = data.filter.state(k,6)+2*pi;
    end
end

filename = path + "reference_vel.txt";
A = importdata(filename, delimiterIn, headerlinesIn);
i = 1;
data.dcl_reference.ros_time = A.data(first:end,i)  - time(1);
data.dcl_reference.time =  A.data(first:end,i+1)  - time(1);
data.dcl_reference.desired_surge = A.data(first:end,i+2);
data.dcl_reference.desired_yaw_rate = A.data(first:end,i+3);

filename = path + "motors.txt";
A = importdata(filename, delimiterIn, headerlinesIn);
i = 1;
data.motors.ros_time = A.data(first:end,i)  - time(1);
data.motors.time = A.data(first:end,i+1)  - time(1);
data.motors.rpm = A.data(first:end,[i+3 i+5]);

filename = path + "thrusters.txt";
A = importdata(filename, delimiterIn, headerlinesIn);
i = 1;
data.thrusters.ros_time = A.data(first:end,i)  - time(1);
data.thrusters.time = A.data(first:end,i+1)  - time(1);
data.thrusters.percentage = A.data(first:end,i+2:i+3);


filename = path + "gps.txt";
A = importdata(filename, delimiterIn, headerlinesIn);
i = 1;
data.gps.ros_time = A.data(first:end,i)  - time(1);
i = 2;
data.gps.time =  A.data(first:end,i)  - time(1);
data.gps.latitude =  A.data(first:end,i+1);
data.gps.longitude =  A.data(first:end,i+2);
data.gps.altitude = A.data(first:end,i+3);
data.gps.speed = A.data(first:end,i+4);
data.gps.cog = A.data(first:end,i+5);
for k=1:length(data.gps.time(:,1))
    [x, y, zone, isnorth] = utmups_fwd(data.gps.latitude(k), data.gps.longitude(k));
    data.gps.ned(k,1) = y;
    data.gps.ned(k,2) = x;
    data.gps.ned(k,3) = data.gps.altitude(k);
end

filename = path + "gps.txt";
A = importdata(filename, delimiterIn, headerlinesIn);
i = 1;
data.gps.ros_time = A.data(first:end,i)  - time(1);
i = 2;
data.gps.time =  A.data(first:end,i)  - time(1);
data.gps.latitude =  A.data(first:end,i+1);
data.gps.longitude =  A.data(first:end,i+2);
data.gps.altitude = A.data(first:end,i+3);
data.gps.speed = A.data(first:end,i+4);
data.gps.cog = A.data(first:end,i+5);
for k=1:length(data.gps.time(:,1))
    [x, y, zone, isnorth] = utmups_fwd(data.gps.latitude(k), data.gps.longitude(k));
    data.gps.ned(k,1) = y;
    data.gps.ned(k,2) = x;
    data.gps.ned(k,3) = data.gps.altitude(k);
end

% using rostime instead of sensortime due to bug
%
filename = path + "sensors.txt";
A = importdata(filename, delimiterIn, headerlinesIn);
data.imu.ros_time = A.data(first:end,1)  - time(1);
data.compass.ros_time = A.data(first:end,1)  - time(1);
data.magnetometer.ros_time = A.data(first:end,1)  - time(1);
i = 2;
data.imu.time = A.data(first:end,i)  - time(1);
data.imu.accelerometer = A.data(first:end,i+1:i+3);
data.imu.gyroscope = A.data(first:end,i+4:i+6);
i = i+7;
data.compass.time = A.data(first:end,i)  - time(1);
data.compass.rpy = A.data(first:end,i+1:i+3);
i = i+4;
data.magnetometer.time = A.data(first:end,i)  - time(1);
data.magnetometer.xyz = A.data(first:end,i+1:i+3);


% using rostime instead of sensortime due to bug
%
filename = path + "stsm_control.txt";
A = importdata(filename, delimiterIn, headerlinesIn);
i = 1;
data.stsm.ros_time = A.data(first:end,i)  - time(1);
data.stsm.time = A.data(first:end,i+1)  - time(1);
data.stsm.sigma_surge = A.data(first:end,i+2);
data.stsm.sigma_yawrate = A.data(first:end,i+3);
data.stsm.tau_surge = A.data(first:end,i+4);
data.stsm.tau_yawrate = A.data(first:end,i+5);


% using rostime instead of sensortime due to bug
%
% filename = path + "pathFollowing.txt";
% i = 1;
% A = importdata(filename, delimiterIn, headerlinesIn);
% data.pathfollow.ros_time = A.data(first:end,i)  - time(1);
% i = 2;
% data.pathfollow.time = A.data(first:end,i) - time(1);
% i = 3;
% data.pathfollow.delta = A.data(first:end,i);
% i = 4;
% data.pathfollow.y = A.data(first:end,i);
% i = 5;
% data.pathfollow.y_real = A.data(first:end,i);
% i = 6;
% data.pathfollow.y_int_dot = A.data(first:end,i);
% i = 7;
% data.pathfollow.y_int = A.data(first:end,i);
% i = 8;
% data.pathfollow.psi = A.data(first:end,i);


if is_sim == 1
    %disp("importing sim data")
    filename = path + "groundtruth.txt";
    A = importdata(filename, delimiterIn, headerlinesIn);
    i = 1;
    data.sim.ros_time = A.data(first:end,i) - time(1);
    i = 2;
    data.sim.time = A.data(first:end,i) - time(1);
    data.sim.state = A.data(first:end,i+1:i+17);
    data.sim.ned = zeros(length(data.sim.state(:,1)),3);
    for k=1:length(data.sim.state(:,1))
        [x, y, zone, isnorth] = utmups_fwd(data.sim.state(k,1), data.sim.state(k,2));
        data.sim.ned(k,1) = y;
        data.sim.ned(k,2) = x;
        data.sim.ned(k,3) = data.sim.state(k,3);
        
        data.sim.worldF_vel(k,:) = rotation(data.sim.state(k,4), data.sim.state(k,5), data.sim.state(k,6))*[data.sim.state(k,7:9)'];
    end
    data.sim.state(:,4) = wrapToPi(data.sim.state(:,4));
    data.sim.state(:,5) = wrapToPi(data.sim.state(:,5));
    
end


% making NED coordinates all relative to the first point
start_point = data.filter.ned(1,:);
%data.sim.ned = data.sim.ned - start_point;
data.filter.ned = data.filter.ned - start_point;
data.gps.ned = data.gps.ned - start_point;


% data.filter.ros_time = data.filter.time;
% data.dcl_reference.ros_time = data.dcl_reference.time;
% data.motors.ros_time = data.motors.time;
% data.thrusters.ros_time = data.thrusters.time;
% data.gps.ros_time = data.gps.time;
% data.imu.ros_time = data.imu.time;
% data.compass.ros_time = data.compass.time;
% data.magnetometer.ros_time = data.magnetometer.time;
end
