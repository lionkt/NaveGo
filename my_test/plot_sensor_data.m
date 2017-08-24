clc;
clear all;
xsens_file_path = 'F:\Momenta_Intern\Data\MTi-G-710数据\4-vehicle_test\8-22-vehicle-test\';
vehicular_file_path = 'F:\momenta文件夹\2017-8-22跑车数据\8-22车载设备数据\';
%%%%%%%%%%%%%%%%%%%%%% xsens数据的提取 %%%%%%%%%%%%%%%%%%%%%
xsens_file_name = 'MT_2017-08-22-20h32-000';
fileID=fopen([xsens_file_path, xsens_file_name,'.txt']);
xsens_data = textscan(fileID,'%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f','HeaderLines',7);
fclose(fileID);
xsens_sample_time_fine = xsens_data{2};
xsens_accX = xsens_data{3};xsens_accY = xsens_data{4};xsens_accZ = xsens_data{5};
xsens_F_accX = xsens_data{6};xsens_F_accY = xsens_data{7};xsens_F_accZ = xsens_data{8};
xsens_gyroX = xsens_data{9};xsens_gyroY = xsens_data{10};xsens_gyroZ = xsens_data{11};
xsens_roll = xsens_data{12};xsens_pitch = xsens_data{13};xsens_yaw = xsens_data{14};
xsens_lat = xsens_data{15};xsens_lon = xsens_data{16};xsens_alt = xsens_data{17};
xsens_velX =  xsens_data{18};xsens_velY =  xsens_data{19};xsens_velZ =  xsens_data{20};
% 删除xsens的NaN的数据
IX = find(isnan(xsens_F_accY) | isnan(xsens_F_accZ));
xsens_sample_time_fine(IX) = [];
xsens_accX(IX) = [];xsens_accY(IX) = [];xsens_accZ(IX) = [];
xsens_F_accX(IX) = [];xsens_F_accY(IX) = [];xsens_F_accZ(IX) = [];
xsens_gyroX(IX) = [];xsens_gyroY(IX) = [];xsens_gyroZ(IX) = [];
xsens_lat(IX) = [];xsens_lon(IX) = [];xsens_alt(IX) = [];
xsens_roll(IX) = [];xsens_pitch(IX) = [];xsens_yaw(IX) = [];
xsens_velX(IX) = [];xsens_velY(IX) = [];xsens_velZ(IX) = [];
xsens_acc_data = [xsens_accX xsens_accY xsens_accZ];
xsens_gyro_data = [xsens_gyroX xsens_gyroY xsens_gyroZ];
xsens_free_acc_data = [xsens_F_accX xsens_F_accY xsens_F_accZ];
xsens_vel_data = [xsens_velX xsens_velY xsens_velZ];

XSENS_SAMPLE_FREQUENCY = 100;     % 采样频率100hz
dt = 1/XSENS_SAMPLE_FREQUENCY;
xsens_sample_time_fine = (xsens_sample_time_fine-xsens_sample_time_fine(1))/10000;        %转换比例为10000 = 1s

NED2ENU = [0,1,0;
           1,0,0;
           0,0,-1];
% vel_data = (NED2ENU*vel_data')';    %NED速度转为ENU速度
xsens_pos_by_vel(:,1) = cumtrapz(xsens_sample_time_fine,xsens_vel_data(:,1));
xsens_pos_by_vel(:,2) = cumtrapz(xsens_sample_time_fine,xsens_vel_data(:,2));
xsens_pos_by_vel(:,3) = cumtrapz(xsens_sample_time_fine,xsens_vel_data(:,3));

%%%%%%%%%%%%%%%%%%%%% 高精度RTK数据的提取 %%%%%%%%%%%%%%%%%%%%%
rtk_name = 'RTK\3.rtk';
% rtk格式：
% "%.2f"%time+' '+"%.7f"%lat+' '+"%.7f"%lon+' '+"%.2f"%alt+' '
%         	+"%.3f"%yaw+' '+"%.3f"%pitch+' '+"%.3f"%roll+' '
%         	+"%.3f"%vE+' '+"%.3f"%vN+' '+"%.3f"%vU+'\n'
fileID = fopen([vehicular_file_path, rtk_name]);
rtk_data = textscan(fileID,'%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f');
rtk_time_tag = rtk_data{1};
rtk_lat = rtk_data{2};rtk_lon = rtk_data{3};
rtk_yaw = rtk_data{5};rtk_pitch = rtk_data{6};rtk_roll = rtk_data{7};
rtk_velX = rtk_data{8};rtk_velY = rtk_data{9};rtk_velZ = rtk_data{10};
rtk_vel = [rtk_velX rtk_velY rtk_velZ];
fclose(fileID);

%%%%%%%%%%%%%%%%%%%%%% laneto数据的提取 %%%%%%%%%%%%%%%%%%%%%
laneto_name = 'LineTo\3.lineTo';
% LaneTo格式：
% str(time)+' '+'%.7f'%lon+' '+'%.7f'%lat+' '+'%.2f'%z+' '+
%             '%.2f'%heading+' '+'%.2f'%pitch+' '+'%.2f'%roll+'\n'
fileID=fopen([vehicular_file_path, laneto_name]);
laneto_data = textscan(fileID,'%f\t%f\t%f\t%f\t%f\t%f\t%f');    
laneto_time_tag = laneto_data{1};
laneto_lat = laneto_data{2};laneto_lon = laneto_data{3};
laneto_yaw = laneto_data{5};laneto_pitch = laneto_data{6};laneto_roll = laneto_data{7};
fclose(fileID);

%%%%%%%%%%%%%%%%%%%%%% single-gps数据的提取 %%%%%%%%%%%%%%%%%%%%%
single_gps_name = 'GPS\3.gps';
fileID=fopen([vehicular_file_path, single_gps_name]);
single_gps_data = textscan(fileID,'%f\t%f\t%f\t%f');    
single_gps_time_stamp = single_gps_data{1};
single_gps_lat = single_gps_data{2};
single_gps_lon = single_gps_data{3};
fclose(fileID);

%% 时间对齐处理
UTC_start_time = datenum('2017-08-20 00:00:00','yyyy-mm-dd HH:MM:SS');
% xsens_start_time_raw = datenum('2017-08-22 12:47:03','yyyy-mm-dd HH:MM:SS');
xsens_start_time_raw = datenum('2017-08-22 12:32:50','yyyy-mm-dd HH:MM:SS');    %MT上读到的时刻是12:32:50
xsens_start_time_utc_std = (xsens_start_time_raw-UTC_start_time)*24*3600;   %把时间转为从UTC周日00:00:00至今秒数
xsens_time_tag_utc_std = (xsens_sample_time_fine + xsens_start_time_utc_std);
disp(['xsens和rtk的起始时刻差(xsens-rtk):',num2str(xsens_start_time_utc_std - rtk_time_tag(1)),'s']);
disp(['xsens和single-gps的起始时刻差(xsens-single_gps):',num2str(xsens_start_time_utc_std - single_gps_time_stamp(1)),'s']);


%% 对rtk和gps数据进行操作
rtk_acc(:,1) = diff(rtk_velX) * (1/dt);
rtk_acc(:,2) = diff(rtk_velY) * (1/dt);
rtk_acc(:,3) = diff(rtk_velZ) * (1/dt);

single_gps_lat1 = single_gps_lat(1:end-1);
single_gps_lat2 = single_gps_lat(2:end);
single_gps_lon1 = single_gps_lon(1:end-1);
single_gps_lon2 = single_gps_lon(2:end);
[arclen,az] = distance(single_gps_lat1,single_gps_lon1,single_gps_lat2,single_gps_lon2,referenceEllipsoid('wgs84'));


%% plot data
% figure;
% plot(xsens_pos_by_vel(:,1),xsens_pos_by_vel(:,2),'.');
% figure;
% plot(xsens_lon,xsens_lat,'.');
figure;
subplot(211);
plot(rtk_time_tag,rtk_velX,'.');
hold on;
plot(xsens_time_tag_utc_std,xsens_vel_data(:,1),'.');
title('x-axis vel');legend('rtk','xsensVel');
subplot(212);
plot(rtk_time_tag,rtk_velY,'.');
hold on;
plot(xsens_time_tag_utc_std,xsens_vel_data(:,2),'.');
title('y-axis vel');legend('rtk','xsensVel');

figure;
subplot(211);
plot(rtk_time_tag,rtk_yaw,'.');
hold on;
plot(xsens_time_tag_utc_std,xsens_yaw,'.');plot(laneto_time_tag,laneto_yaw,'.');
title('yaw');legend('rtk','xsens','laneto');
subplot(212);
plot(rtk_time_tag,rtk_pitch,'.');
hold on;
plot(xsens_time_tag_utc_std,xsens_pitch,'.');plot(laneto_time_tag,laneto_pitch,'.');
title('pitch');legend('rtk','xsens','laneto');

figure;
subplot(211);
plot(rtk_time_tag(1:end-1),rtk_acc(:,1)); 
hold on;
plot(xsens_time_tag_utc_std,xsens_free_acc_data(:,1));
title('x-axis free-acc');legend('rtk','xsens free');

figure;
subplot(211);
plot(single_gps_lat,single_gps_lon); 
hold on;
plot(xsens_lat,xsens_lon);
title('location');legend('single-gps','xsens');
