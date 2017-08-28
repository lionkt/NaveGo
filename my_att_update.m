function [ qua_n, DCMbn_n, euler ] = my_att_update( wb, fb, DCMbn, qua, omega_ie_N, omega_en_N, dt, att_mode )
%   根据6轴融合的方法计算姿态更新
%   crwon add，原来Navego中的姿态更新方法效果不够好
% INPUT:
%   wb,         3x1 incremental turn rates in body-frame (rad/s).
%   fb,         3x1 acceleration in body-frame (m/s^2).
%   DCMbn,      3x3 body-to-nav DCM.
%   qua,        4x1 quaternion.
%   omega_ie_N, 3x1 Earth rate (rad/s).
%   omega_en_N, 3x1 Transport rate (rad/s).
%   dt,         1x1 INS time period (s).
%	att_mode,   attitude mode string.
%      'quaternion': attitude updated in quaternion format. Default value.
%             'dcm': attitude updated in Direct Cosine Matrix format.
%
% OUTPUT:
%   qua_n,      4x1 updated quaternion.
%   DCMbn_n,    3x3 updated body-to-nav DCM.
%   euler,      3x1 updated Euler angles (rad).
%

if nargin < 7, att_mode  = 'quaternion'; end

%% 根据6轴融合的方法，更新姿态
Ki = 0.01;     % PID控制器的增益
Kp = 0.1;   % PID控制器的增益
exInt = 0.0;eyInt = 0.0;ezInt = 0.0;
ax = fb(1);ay = fb(2);az = fb(3);
gx = wb(1);gy = wb(2);gz = wb(3);
q0 = qua(1);q1 = qua(2);q2 = qua(3);q3 = qua(4);
% normalise the measurements
norm = sqrt(ax*ax + ay*ay + az*az); 
ax = ax / norm;
ay = ay / norm;
az = az / norm; 

%%% Note1: estimated direction of gravity
vx = 2*(q1*q3 - q0*q2);
vy = 2*(q0*q1 + q2*q3);
vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
%%% Note2: error is sum of cross product between reference direction of field and direction measured by sensor
ex = (ay*vz - az*vy);
ey = (az*vx - ax*vz);
ez = (ax*vy - ay*vx);
% integral error scaled integral gain
exInt = exInt + ex*Ki;  
eyInt = eyInt + ey*Ki;
ezInt = ezInt + ez*Ki;
% adjusted gyroscope measurements， PID 
gx = gx + Kp*ex + exInt;
gy = gy + Kp*ey + eyInt;
gz = gz + Kp*ez + ezInt;

q0temp=q0; 
q1temp=q1; 
q2temp=q2;
q3temp=q3;

%%% Note3: integrate quaternion rate and normalise
q0 = q0temp + (-q1temp *gx - q2temp *gy - q3temp *gz)*(dt/2);
q1 = q1temp + (q0temp *gx + q2temp *gz - q3temp *gy)*(dt/2);
q2 = q2temp + (q0temp *gy - q1temp *gz + q3temp *gx)*(dt/2);
q3 = q3temp + (q0temp *gz + q1temp *gy - q2temp *gx)*(dt/2); 
% normalise quaternion
norm = 1.0/sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
q0 = q0 * norm;
q1 = q1 * norm;
q2 = q2 * norm;
q3 = q3 * norm;


%% 根据输入的mode选择姿态更新方式
if strcmp(att_mode, 'quaternion')           % Quaternion update 
%     qua_n   = qua_update(qua, wb_n, dt);    % Update quaternions
%     qua_n   = qua_n / norm(qua_n);          % Brute-force normalization
    qua_n   = [q0;q1;q2;q3];                
    DCMbn_n = qua2dcm(qua_n);               % Update DCM
    euler   = qua2euler(qua_n);             % Update Euler angles
elseif strcmp(att_mode, 'dcm')              % DCM update  
    error('att_update: do not support DCM updating now.');
%     euler_i = wb_n * dt;                    % Incremental Euler angles 
%     DCMbn_n = dcm_update(DCMbn, euler_i);   % Update DCM
%     euler   = dcm2euler(DCMbn_n);           % Update Euler angles
%     qua_n   = euler2qua(euler);             % Update quaternions
%     qua_n   = qua_n / norm(qua_n);          % Brute-force normalization
    
else
    error('att_update: no attitude update mode defined.')
end


end

