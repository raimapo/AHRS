% ExampleScript.m
%
% This script demonstrates use of the MadgwickAHRS and MahonyAHRS algorithm
% classes with example data. ExampleData.mat contains calibrated gyroscope,
% accelerometer and magnetometer data logged from an AHRS device (x-IMU)
% while it was sequentially rotated from 0 degrees, to +90 degree and then
% to -90 degrees around the X, Y and Z axis.  The script first plots the
% example sensor data, then processes the data through the algorithm and
% plots the output as Euler angles.
%
% Note that the Euler angle plot shows erratic behaviour in phi and psi
% when theta approaches �90 degrees. This due to a singularity in the Euler
% angle sequence known as 'Gimbal lock'.  This issue does not exist for a
% quaternion or rotation matrix representation.
%
% Date          Author          Notes
% 28/09/2011    SOH Madgwick    Initial release
% 13/04/2012    SOH Madgwick    deg2rad function no longer used
% 06/11/2012    Seb Madgwick    radian to degrees calculation corrected
%
% 07/05/2018    Raimondas Pomarnacki     Added Gyro to Euler. Used Phil Kim books, rotation sequence changed to ZYX 
% 10/05/2018    Raimondas Pomarnacki     Added Accel and Mag to Euler. Used Phil Kim books, rotation sequence changed to ZYX and added YAW/Heading/Psi calculation                                     
%                                       
%                                       Linear Kalman filter to Euler
%                                       Added EKF 9DOF to Euler
%


%% Start of script

addpath('quaternion_library');      % include quaternion library
addpath('GyroscopeIntegration');    % include Gyroscope integration library
addpath('AccelerometerMagnetometer');    % include Accelerometer and Magnetometer Integration library
addpath('EulerEKF');    % include Gyroscope integration library
close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal

%% Import and plot sensor data

load('ExampleData.mat');

figure('Name', 'Sensor Data');
axis(1) = subplot(3,1,1);
hold on;
plot(time, Gyroscope(:,1), 'r');
plot(time, Gyroscope(:,2), 'g');
plot(time, Gyroscope(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Angular rate (deg/s)');
title('Gyroscope');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(time, Accelerometer(:,1), 'r');
plot(time, Accelerometer(:,2), 'g');
plot(time, Accelerometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Acceleration (g)');
title('Accelerometer');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(time, Magnetometer(:,1), 'r');
plot(time, Magnetometer(:,2), 'g');
plot(time, Magnetometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Flux (G)');
title('Magnetometer');
hold off;
linkaxes(axis, 'x');

%% Process sensor data through Madgwic algorithm

AHRS = MadgwickAHRS('SamplePeriod', 1/256, 'Beta', 0.1);

quaternion = zeros(length(time), 4);
for t = 1:length(time)
    AHRS.Update(Gyroscope(t,:) * (pi/180), Accelerometer(t,:), Magnetometer(t,:));	% gyroscope units must be radians
    quaternion(t, :) = AHRS.Quaternion;
end

%% Plot algorithm output as Euler angles
% The first and third Euler angles in the sequence (phi and psi) become
% unreliable when the middle angles of the sequence (theta) approaches �90
% degrees. This problem commonly referred to as Gimbal Lock.
% See: http://en.wikipedia.org/wiki/Gimbal_lock

euler = quatern2euler(quaternConj(quaternion)) * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.

figure('Name', 'MadgwickAHRS Euler Angles');
hold on;
plot(time, euler(:,1), 'r');
plot(time, euler(:,2), 'g');
plot(time, euler(:,3), 'b');
title('MadgwickAHRS Euler angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi');
hold off;

%% Process sensor data through Mahony algorithm

AHRS = MahonyAHRS('SamplePeriod', 1/256, 'Kp', 0.5);

quaternion = zeros(length(time), 4);
for t = 1:length(time)
    AHRS.Update(Gyroscope(t,:) * (pi/180), Accelerometer(t,:), Magnetometer(t,:));	% gyroscope units must be radians
    quaternion(t, :) = AHRS.Quaternion;
end

%% Plot algorithm output as Euler angles
% The first and third Euler angles in the sequence (phi and psi) become
% unreliable when the middle angles of the sequence (theta) approaches �90
% degrees. This problem commonly referred to as Gimbal Lock.
% See: http://en.wikipedia.org/wiki/Gimbal_lock

euler = quatern2euler(quaternConj(quaternion)) * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.

figure('Name', 'MahonyAHRS Euler Angles');
hold on;
plot(time, euler(:,1), 'r');
plot(time, euler(:,2), 'g');
plot(time, euler(:,3), 'b');
title('MahonyAHRS Euler angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi');
hold off;

%% Process sensor data through Gyrocope angular rate integration algorithm

EulerSaved = zeros(length(time), 3);

for t = 1:length(time)
    if t > 1
        dt = (time(t)-time(t-1));
    else
        dt = (time(2)-time(1));
    end
    
    %degree/s to rad/s
    p = Gyroscope(t, 1) * pi/180;
    q = Gyroscope(t, 2) * pi/180;
    r = Gyroscope(t, 3) * pi/180;
     
    [phi, theta, psi] = EulerGyro(p, q, r, dt); 
  
    EulerSaved(t, :) = [ phi theta psi ];
    
end

PhiSaved   = EulerSaved(:, 1) * 180/pi;
ThetaSaved = EulerSaved(:, 2) * 180/pi;
PsiSaved   = EulerSaved(:, 3) * 180/pi;


figure('Name', 'Gysoscope angular rate integration to Euler Angles');
hold on;
plot(time, PhiSaved, 'r');
plot(time, ThetaSaved, 'g');
plot(time, PsiSaved, 'b');
title('Gysoscope angular rate integration to Euler Angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi');
grid on;
hold off;

%% Process sensor data through Accelerometer and Magnetometer integration algorithm

EulerSaved = zeros(length(time), 3);

for t = 1:length(time)
    if t > 1
        dt = (time(t)-time(t-1));
    else
        dt = (time(2)-time(1));
    end
             
    
    [phi, theta, psi] = EulerAccel(Accelerometer(t, 1), Accelerometer(t, 2), Magnetometer(t,1), Magnetometer(t,2)); 

    EulerSaved(t, :) = [ phi theta psi ];

end

PhiSaved   = EulerSaved(:, 1) * 180/pi;
ThetaSaved = EulerSaved(:, 2) * 180/pi;
PsiSaved   = EulerSaved(:, 3) * 180/pi;

figure('Name', 'Accelerometer and Magnetometer to Euler Angles');
hold on;
plot(time, PhiSaved, 'r');
plot(time, ThetaSaved, 'g');
plot(time, PsiSaved, 'b');
title('Accelerometer and Magnetometer to Euler Angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi');
grid on;
hold off;

%% End of script