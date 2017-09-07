% Example of use of NaveGo.
% 
% Main goal: to compare two INS/GPS systems performances, one using a 
% simulated ADIS16405 IMU and simulated GPS, and another using a 
% simulated ADIS16488 IMU and the same simulated GPS.
%
%   Copyright (C) 2014, Rodrigo Gonzalez, all rights reserved.
%
%   This file is part of NaveGo, an open-source MATLAB toolbox for
%   simulation of integrated navigation systems.
%
%   NaveGo is free software: you can redistribute it and/or modify
%   it under the terms of the GNU Lesser General Public License (LGPL)
%   version 3 as published by the Free Software Foundation.
%
%   This program is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%   GNU Lesser General Public License for more details.
%
%   You should have received a copy of the GNU Lesser General Public
%   License along with this program. If not, see
%   <http://www.gnu.org/licenses/>.
%
% References:
%           R. Gonzalez, J. Giribet, and H. Pati帽o. NaveGo: a
% simulation framework for low-cost integrated navigation systems,
% Journal of Control Engineering and Applied Informatics, vol. 17,
% issue 2, pp. 110-120, 2015.
%
%           Analog Devices. ADIS16400/ADIS16405 datasheet. High Precision 
% Tri-Axis Gyroscope, Accelerometer, Magnetometer. Rev. B. 
% http://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16400_16405.pdf
%
%           Analog Devices. ADIS16488 datasheet. Tactical Grade Ten Degrees 
% of Freedom Inertial Sensor. Rev. G. 
% http://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16488.pdf
%
%			Garmin International, Inc. GPS 18x TECHNICAL SPECIFICATIONS.
% Revision D. October 2011. 
% http://static.garmin.com/pumac/GPS_18x_Tech_Specs.pdf
% 
% Version: 010
% Date:    2017/05/15
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

clc
close all
clear
matlabrc

versionstr = 'NaveGo, release v0.8.0-alpha';

fprintf('\n%s.\n', versionstr)
fprintf('\nNaveGo: starting simulation ... \n')

%% CODE EXECUTION PARAMETERS

% Comment any of the following parameters in order to NOT execute a particular portion of code

GPS_DATA  = 'ON';   % Simulate GPS data
IMU1_DATA = 'ON';   % Simulate ADIS16405 IMU data
IMU2_DATA = 'OFF';   % Simulate ADIS16488 IMU data

IMU1_INS  = 'ON';   % Execute INS/GPS integration for ADIS16405 IMU
IMU2_INS  = 'OFF';   % Execute INS/GPS integration for ADIS16488 IMU

PLOT      = 'ON';   % Plot results.

% If a particular parameter is commented above, it is set by default to 'OFF'.

if (~exist('GPS_DATA','var')),  GPS_DATA  = 'OFF'; end
if (~exist('IMU1_DATA','var')), IMU1_DATA = 'OFF'; end
if (~exist('IMU2_DATA','var')), IMU2_DATA = 'OFF'; end
if (~exist('IMU1_INS','var')),  IMU1_INS = 'OFF'; end
if (~exist('IMU2_INS','var')),  IMU2_INS = 'OFF'; end
if (~exist('PLOT','var')),      PLOT     = 'OFF'; end

%% CONVERSION CONSTANTS

G = 9.80151;           % Gravity constant, m/s^2
% G = 9.80151;           % Gravity constant, m/s^2
G2MSS = G;          % g to m/s^2
MSS2G = (1/G);      % m/s^2 to g

D2R = (pi/180);     % degrees to radians
R2D = (180/pi);     % radians to degrees

KT2MS = 0.514444;   % knot to m/s
MS2KMH = 3.6;       % m/s to km/h

%% LOAD REFERENCE DATA

fprintf('NaveGo: loading reference dataset from a rtk file... \n')

load('./my_test/ref_rtk.mat');

% ref.mat contains the reference data structure from which inertial 
% sensors and GPS wil be simulated. It must contain the following fields:

%         t: Nx1 time vector (seconds).
%       lat: Nx1 latitude (radians).
%       lon: Nx1 longitude (radians).
%         h: Nx1 altitude (m).
%       vel: Nx3 NED velocities (m/s).
%      roll: Nx1 roll angles (radians).
%     pitch: Nx1 pitch angles (radians).
%       yaw: Nx1 yaw angle vector (radians).
%        kn: 1x1 number of elements of time vector.
%     DCMnb: Nx9 Direct Cosine Matrix nav-to-body. Each row contains 
%            the elements of one matrix ordered by columns as 
%            [a11 a21 a31 a12 a22 a32 a13 a23 a33].
%      freq: sampling frequency (Hz).

%% ADIS16405 IMU error profile

% IMU data structure:
%         t: Ix1 time vector (seconds).
%        fb: Ix3 accelerations vector in body frame XYZ (m/s^2).
%        wb: Ix3 turn rates vector in body frame XYZ (radians/s).
%       arw: 1x3 angle random walks (rad/s/root-Hz).
%       vrw: 1x3 angle velocity walks (m/s^2/root-Hz).
%      gstd: 1x3 gyros standard deviations (radians/s).
%      astd: 1x3 accrs standard deviations (m/s^2).
%    gb_fix: 1x3 gyros static biases or turn-on biases (radians/s).
%    ab_fix: 1x3 accrs static biases or turn-on biases (m/s^2).
%  gb_drift: 1x3 gyros dynamic biases or bias instabilities (radians/s).
%  ab_drift: 1x3 accrs dynamic biases or bias instabilities (m/s^2).
%   gb_corr: 1x3 gyros correlation times (seconds).
%   ab_corr: 1x3 accrs correlation times (seconds).
%     gpsd : 1x3 gyros dynamic biases PSD (rad/s/root-Hz).
%     apsd : 1x3 accrs dynamic biases PSD (m/s^2/root-Hz);
%      freq: 1x1 sampling frequency (Hz).
% ini_align: 1x3 initial attitude at t(1).
% ini_align_err: 1x3 initial attitude errors at t(1).

%% MTi-G-710 IMU error profile
MTiG710.arw      = 0.3  .* ones(1,3);     % Angle random walks [X Y Z] (deg/root-hour)
MTiG710.vrw      = 0.029.* ones(1,3);     % Velocity random walks [X Y Z] (m/s/root-hour)
MTiG710.gb_fix   = 0.2  .* ones(1,3);     % Gyro static biases [X Y Z] (deg/s)
% MTiG710.ab_fix   = 16   .* ones(1,3);     % Acc static biases [X Y Z] (mg)
MTiG710.ab_fix   = [6, 6, 10];     % Acc static biases [X Y Z] (mg)
MTiG710.gb_drift = 6.5/3600  .* ones(1,3);% Gyro dynamic biases [X Y Z] (deg/s)
MTiG710.ab_drift = 0.1  .* ones(1,3);     % Acc dynamic biases [X Y Z] (mg)
MTiG710.gb_corr  = 100  .* ones(1,3);     % Gyro correlation times [X Y Z] (seconds)
MTiG710.ab_corr  = 100  .* ones(1,3);     % Acc correlation times [X Y Z] (seconds)
MTiG710.freq     = 100;              % IMU operation frequency [X Y Z] (Hz)
% MTiG710.m_psd = 0.054 .* ones(1,3);       % Magnetometer noise density [X Y Z] (mgauss/root-Hz)

% ref dataset will be used to simulate IMU sensors.
fprintf('NaveGo: loading xsens data... \n') 
load('./my_test/xsens_imu.mat');
dt = mean(diff(xsens_imu.t));               % IMU mean period
t = xsens_imu.t;
fb = xsens_imu.fb;
wb = xsens_imu.wb;
xsens_imu = imu_err_profile(MTiG710, dt);      % Transform IMU manufacturer error units to SI units.
xsens_imu.t = t;
xsens_imu.fb = fb;
xsens_imu.wb = wb;
% xsens_imu.ini_align_err = -[5.828975 -1.104427 -91.761559] .* D2R;      % Initial attitude align errors for matrix P in Kalman filter, [roll pitch yaw] (radians)  
% xsens_imu.ini_align = [ref_rtk.roll(1) ref_rtk.pitch(1) ref_rtk.yaw(1)];  % Initial attitude align at t(1) (radians). 这个数据直接从XSENS给出的attitude读出来
xsens_imu.ini_align_err = [1 1 5] .* D2R;      % Initial attitude align errors for matrix P in Kalman filter, [roll pitch yaw] (radians)  
xsens_imu.ini_align = -[5.828975 -1.104427 -91.761559] .* D2R;  % Initial attitude align at t(1) (radians). 这个数据直接从XSENS给出的attitude读出来

%% Garmin 5-18 Hz GPS error profile

% GPS data structure:
%         t: Mx1 time vector (seconds).
%       lat: Mx1 latitude (radians).
%       lon: Mx1 longitude (radians).
%         h: Mx1 altitude (m).
%       vel: Mx3 NED velocities (m/s).
%       std: 1x3 position standard deviations (rad, rad, m).
%      stdm: 1x3 position standard deviations (m, m, m).
%      stdv: 1x3 velocity standard deviations (m/s).
%      larm: 3x1 lever arm (x-right, y-fwd, z-down) (m).
%      freq: 1x1 sampling frequency (Hz).

single_gps.stdm = [5, 5, 10];                 % GPS positions standard deviations [lat lon h] (meters)
single_gps.stdv = 0.1 * KT2MS .* ones(1,3);   % GPS velocities standard deviations [Vn Ve Vd] (meters/s)
single_gps.larm = zeros(3,1);                 % GPS lever arm [X Y Z] (meters)
single_gps.freq = 1;                          % GPS operation frequency (Hz)

fprintf('NaveGo: loading GPS data... \n') 
load('./my_test/single_gps.mat');
% single_gps = gps_err_profile(single_gps.lat(1), single_gps.h(1), single_gps); % Transform GPS manufacturer error units to SI units.
single_gps = gps_err_profile(ref_rtk.lat(1), ref_rtk.h(1), single_gps); % Transform GPS manufacturer error units to SI units.
%% SIMULATE GPS


%% SIMULATE IMU1


%% INS/GPS integration using IMU1

if strcmp(IMU1_INS, 'ON')
    
    fprintf('NaveGo: INS/GPS integration for IMU1... \n')
    
    % Sincronize GPS data with IMU data.
    
    % Guarantee that gps.t(1) < imu1.t(1) < gps.t(2)
    if (xsens_imu.t(1) < single_gps.t(1)),
        
        igx  = find(xsens_imu.t > single_gps.t(1), 1, 'first' );
        
        xsens_imu.t  = xsens_imu.t  (igx:end, :);
        xsens_imu.fb = xsens_imu.fb (igx:end, :);
        xsens_imu.wb = xsens_imu.wb (igx:end, :);        
    end
    gps1 = single_gps;
    if(xsens_imu.t(1) >= single_gps.t(1))
        igx = find(single_gps.t < xsens_imu.t(1),1,'last');
        if isnan(igx)
            igx=1;
        end
        gps1.t   = single_gps.t  (igx:end, :);
        gps1.lat = single_gps.lat(igx:end, :);
        gps1.lon = single_gps.lon(igx:end, :);
        gps1.h   = single_gps.h  (igx:end, :);
        gps1.vel = single_gps.vel(igx:end, :);
    end
    
    % Guarantee that imu1.t(end-1) < gps.t(end) < imu1.t(end)    
    if (xsens_imu.t(end) <= single_gps.t(end)),
        
        fgx  = find(single_gps.t < xsens_imu.t(end), 1, 'last' );
        
        gps1.t   = single_gps.t  (1:fgx, :);
        gps1.lat = single_gps.lat(1:fgx, :);
        gps1.lon = single_gps.lon(1:fgx, :);
        gps1.h   = single_gps.h  (1:fgx, :);
        gps1.vel = single_gps.vel(1:fgx, :);
    end
    
    % Execute INS/GPS integration
    % ---------------------------------------------------------------------
    [imu1_e] = ins_gps(xsens_imu, gps1, 'quaternion', 'double');
    % ---------------------------------------------------------------------
    
    save imu1_e.mat imu1_e
    
else
    
    fprintf('NaveGo: loading INS/GPS integration for IMU1... \n')
    
    load imu1_e.mat
end

%% Interpolate INS/GPS dataset 

% INS/GPS estimates and GPS data are interpolated according to the
% reference dataset.

[imu1_ref, ref_1] = navego_interpolation (imu1_e, ref_rtk);
[gps_ref, ref_g] = navego_interpolation (single_gps, ref_rtk);

%% Print navigation time

to = (ref_rtk.t(end) - ref_rtk.t(1));

fprintf('\nNaveGo: navigation time is %.2f minutes or %.2f seconds. \n', (to/60), to)

%% Print RMSE from IMU1

print_rmse (imu1_ref, gps_ref, ref_1, ref_g, 'INS/GPS IMU1');

%% 存储attitude数据，方便对比
imu_e_time_for_save = imu1_e.t;
imu_e_att_for_save = [imu1_e.roll, imu1_e.pitch, imu1_e.yaw];
save('./calculation_data/imu_e_time','imu_e_time_for_save');
save('./calculation_data/imu_e_att','imu_e_att_for_save');

%% PLOT

if (strcmp(PLOT,'ON'))
    
    sig3_rr = abs(imu1_e.Pp(:, 1:22:end).^(0.5)) .* 3; % Only take diagonal elements from Pp
    
    % TRAJECTORY
    figure;
    plot3(ref_rtk.lon.*R2D, ref_rtk.lat.*R2D, ref_rtk.h)
    hold on
    plot3(imu1_e.lon.*R2D, imu1_e.lat.*R2D, imu1_e.h,'.')    
    legend('ref','imu estimation');
    plot3(ref_rtk.lon(1).*R2D, ref_rtk.lat(1).*R2D, ref_rtk.h(1), 'or', 'MarkerSize', 10, 'LineWidth', 2)
    plot3(imu1_e.lon(1).*R2D, imu1_e.lat(1).*R2D, imu1_e.h(1), 'or', 'MarkerSize', 10, 'LineWidth', 2)
    axis tight
    grid on; 
    title('TRAJECTORY')
    xlabel('Longitude [deg.]')
    ylabel('Latitude [deg.]')
    zlabel('Altitude [m]')
    
    % 2D-TRAJECTORY
    figure;
    mstruct = defaultm('mercator');
    mstruct.origin = [39.999374 116.340673 41];
    mstruct = defaultm(mstruct);
    [xxx,yyy] = mfwdtran(mstruct,ref_rtk.lat,ref_rtk.lon);
    plot(xxx, yyy)
%     plot(ref_rtk.lon.*R2D, ref_rtk.lat.*R2D)
    hold on
    [xxx,yyy] = mfwdtran(mstruct,imu1_e.lat,imu1_e.lon);
    plot(xxx, yyy,'.')
%     plot(imu1_e.lon.*R2D, imu1_e.lat.*R2D,'.')
    [xxx,yyy] = mfwdtran(mstruct,single_gps.lat,single_gps.lon);
    plot(xxx, yyy,'o')
%     plot(single_gps.lon.*R2D,single_gps.lat.*R2D,'o');
    %%%%%%% 在gps信号到来的位置，加入速度偏置修正，防止imu被gps拉的不连续 %%%%%%%
    IXX = find_nears_time(imu1_e.t, single_gps.t);
    
    [xxx,yyy] = mfwdtran(mstruct,imu1_e.lat(IXX),imu1_e.lon(IXX));
    plot(xxx, yyy,'d')
%     plot(imu1_e.lon(IXX).*R2D, imu1_e.lat(IXX).*R2D,'d')
    legend('ref','imu estimation','singleGps','imu-estimation(GPS comes)');
    [xxx,yyy] = mfwdtran(mstruct,ref_rtk.lat(1),ref_rtk.lon(1));
    plot(xxx, yyy, 'or', 'MarkerSize', 10, 'LineWidth', 2)
    [xxx,yyy] = mfwdtran(mstruct,imu1_e.lat(1),imu1_e.lon(1));
    plot(xxx, yyy, 'or', 'MarkerSize', 10, 'LineWidth', 2)
%     plot(ref_rtk.lon(1).*R2D, ref_rtk.lat(1).*R2D, 'or', 'MarkerSize', 10, 'LineWidth', 2)
%     plot(imu1_e.lon(1).*R2D, imu1_e.lat(1).*R2D, 'or', 'MarkerSize', 10, 'LineWidth', 2)
    axis tight
    axis equal
    grid on; 
    title('TRAJECTORY')
    xlabel('x-axis')
    ylabel('y-axis')
    
    
    % ATTITUDE
    figure;
    subplot(311)
    plot(ref_rtk.t, R2D.*ref_rtk.roll, '--k', imu1_e.t, R2D.*imu1_e.roll,'-b');
    ylabel('[deg]')
    xlabel('Time [s]')
    legend('REF', 'IMU1');
    title('ROLL');
    
    subplot(312)
    plot(ref_rtk.t, R2D.*ref_rtk.pitch, '--k', imu1_e.t, R2D.*imu1_e.pitch,'-b');
    ylabel('[deg]')
    xlabel('Time [s]')
    legend('REF', 'IMU1');
    title('PITCH');
    
    subplot(313)
    plot(ref_rtk.t, R2D.* ref_rtk.yaw, '--k', imu1_e.t, R2D.*imu1_e.yaw,'-b');
    ylabel('[deg]')
    xlabel('Time [s]')
    legend('REF', 'IMU1');
    title('YAW');
    
    % ATTITUDE ERRORS
    figure;
    subplot(311)
    plot(imu1_ref.t, (imu1_ref.roll-ref_1.roll).*R2D, '-b');
    hold on
    plot (gps1.t, R2D.*sig3_rr(:,1), '--k', gps1.t, -R2D.*sig3_rr(:,1), '--k' )
    ylabel('[deg]')
    xlabel('Time [s]')
    legend('IMU1', '3\sigma');
    title('ROLL ERROR');
    
    subplot(312)
    plot(imu1_ref.t, (imu1_ref.pitch-ref_1.pitch).*R2D, '-b');
    hold on
    plot (gps1.t, R2D.*sig3_rr(:,2), '--k', gps1.t, -R2D.*sig3_rr(:,2), '--k' )
    ylabel('[deg]')
    xlabel('Time [s]')
    legend('IMU1', '3\sigma');
    title('PITCH ERROR');
    
    subplot(313)
    plot(imu1_ref.t, (imu1_ref.yaw-ref_1.yaw).*R2D, '-b');
    hold on
    plot (gps1.t, R2D.*sig3_rr(:,3), '--k', gps1.t, -R2D.*sig3_rr(:,3), '--k' )
    ylabel('[deg]')
    xlabel('Time [s]')
    legend('IMU1', '3\sigma');
    title('YAW ERROR');
    
    % VELOCITIES
    figure;
    subplot(311)
    plot(ref_rtk.t, ref_rtk.vel(:,1), '--k', gps1.t, gps1.vel(:,1),'-c', imu1_e.t, imu1_e.vel(:,1),'-b');
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('REF', 'GPS', 'IMU1');
    title('NORTH VELOCITY');
    
    subplot(312)
    plot(ref_rtk.t, ref_rtk.vel(:,2), '--k', gps1.t, gps1.vel(:,2),'-c', imu1_e.t, imu1_e.vel(:,2),'-b');
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('REF', 'GPS', 'IMU1');
    title('EAST VELOCITY');
    
    subplot(313)
    plot(ref_rtk.t, ref_rtk.vel(:,3), '--k', gps1.t, gps1.vel(:,3),'-c', imu1_e.t, imu1_e.vel(:,3),'-b');
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('REF', 'GPS', 'IMU1');
    title('DOWN VELOCITY');
    
    % VELOCITIES ERRORS
    figure;
    subplot(311)
    plot(gps_ref.t, (gps_ref.vel(:,1) - ref_g.vel(:,1)), '-c');
    hold on
    plot(imu1_ref.t, (imu1_ref.vel(:,1) - ref_1.vel(:,1)), '-b');
    hold on
    plot (gps1.t, sig3_rr(:,4), '--k', gps1.t, -sig3_rr(:,4), '--k' )
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('GPS', 'IMU1', '3\sigma');
    title('VELOCITY NORTH ERROR');
    
    subplot(312)
    plot(gps_ref.t, (gps_ref.vel(:,2) - ref_g.vel(:,2)), '-c');
    hold on
    plot(imu1_ref.t, (imu1_ref.vel(:,2) - ref_1.vel(:,2)), '-b');
    hold on
    plot (gps1.t, sig3_rr(:,5), '--k', gps1.t, -sig3_rr(:,5), '--k' )
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('GPS', 'IMU1', '3\sigma');
    title('VELOCITY EAST ERROR');
    
    subplot(313)
    plot(gps_ref.t, (gps_ref.vel(:,3) - ref_g.vel(:,3)), '-c');
    hold on
    plot(imu1_ref.t, (imu1_ref.vel(:,3) - imu1_ref.vel(:,3)), '-b');
    hold on
    plot (gps1.t, sig3_rr(:,6), '--k', gps1.t, -sig3_rr(:,6), '--k' )
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('GPS', 'IMU1', '3\sigma');
    title('VELOCITY DOWN ERROR');
    
    % POSITION
    figure;
    subplot(311)
    plot(ref_rtk.t, ref_rtk.lat .*R2D, '--k', gps1.t, gps1.lat.*R2D, '-c', imu1_e.t, imu1_e.lat.*R2D, '-b');
    xlabel('Time [s]')
    ylabel('[deg]')
    legend('REF', 'GPS', 'IMU1');
    title('LATITUDE');
    
    subplot(312)
    plot(ref_rtk.t, ref_rtk.lon .*R2D, '--k', gps1.t, gps1.lon.*R2D, '-c', imu1_e.t, imu1_e.lon.*R2D, '-b');
    xlabel('Time [s]')
    ylabel('[deg]')
    legend('REF', 'GPS', 'IMU1');
    title('LONGITUDE');
    
    subplot(313)
    plot(ref_rtk.t, ref_rtk.h, '--k', gps1.t, gps1.h, '-c', imu1_e.t, imu1_e.h, '-b');
    xlabel('Time [s]')
    ylabel('[m]')
    legend('REF', 'GPS', 'IMU1');
    title('ALTITUDE');
    
    % POSITION ERRORS
    % fh = @radicurv;
    % [RNs,REs] = arrayfun(fh, lat_rs);
    
    [RN,RE]  = radius(imu1_ref.lat, 'double');
    LAT2M = RN + imu1_ref.h;
    LON2M = (RE + imu1_ref.h).*cos(imu1_ref.lat);
    
    [RN,RE]  = radius(gps1.lat, 'double');
    LAT2M_G = RN + gps1.h;
    LON2M_G = (RE + gps1.h).*cos(gps1.lat);
    
    [RN,RE]  = radius(gps_ref.lat, 'double');
    LAT2M_GR = RN + gps_ref.h;
    LON2M_GR = (RE + gps_ref.h).*cos(gps_ref.lat);
    
    figure;
    subplot(311)
    plot(gps_ref.t,  LAT2M_GR.*(gps_ref.lat - ref_g.lat), '-c')
    hold on
    plot(imu1_ref.t, LAT2M.*(imu1_ref.lat - ref_1.lat), '-b')
    hold on
    plot (gps1.t, LAT2M_G.*sig3_rr(:,7), '--k', gps1.t, -LAT2M_G.*sig3_rr(:,7), '--k' )
    xlabel('Time [s]')
    ylabel('[m]')
    legend('GPS', 'IMU1', '3\sigma');
    title('LATITUDE ERROR');
    
    subplot(312)
    plot(gps_ref.t, LON2M_GR.*(gps_ref.lon - ref_g.lon), '-c')
    hold on
    plot(imu1_ref.t, LON2M.*(imu1_ref.lon - ref_1.lon), '-b')
    hold on
    plot(gps1.t, LON2M_G.*sig3_rr(:,8), '--k', gps1.t, -LON2M_G.*sig3_rr(:,8), '--k' )
    xlabel('Time [s]')
    ylabel('[m]')
    legend('GPS', 'IMU1', '3\sigma');
    title('LONGITUDE ERROR');
    
    subplot(313)
    plot(gps_ref.t, (gps_ref.h - ref_g.h), '-c')
    hold on
    plot(imu1_ref.t, (imu1_ref.h - ref_1.h), '-b')
    hold on
    plot(gps1.t, sig3_rr(:,9), '--k', gps1.t, -sig3_rr(:,9), '--k' )
    xlabel('Time [s]')
    ylabel('[m]')
    legend('GPS', 'IMU1', '3\sigma');
    title('ALTITUDE ERROR');
    
end
