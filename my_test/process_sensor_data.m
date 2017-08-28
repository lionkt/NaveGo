clc;
clear all;
xsens_file_path = 'F:\Momenta_Intern\Data\MTi-G-710����\4-vehicle_test\8-22-vehicle-test\';
vehicular_file_path = 'F:\momenta�ļ���\2017-8-22�ܳ�����\8-22�����豸����\';
%%%%%%%%%%%%%%%%%%%%%% xsens���ݵ���ȡ %%%%%%%%%%%%%%%%%%%%%
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
% ɾ��xsens��NaN������
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
xsens_att = [xsens_roll xsens_pitch xsens_yaw];
xsens_free_acc_data = [xsens_F_accX xsens_F_accY xsens_F_accZ];
xsens_vel_data = [xsens_velX xsens_velY xsens_velZ];

XSENS_SAMPLE_FREQUENCY = 100;     % ����Ƶ��100hz
dt = 1/XSENS_SAMPLE_FREQUENCY;
xsens_sample_time_fine = (xsens_sample_time_fine-xsens_sample_time_fine(1))/10000;        %ת������Ϊ10000 = 1s

NED2ENU = [0,1,0;
           1,0,0;
           0,0,-1];
% vel_data = (NED2ENU*vel_data')';    %NED�ٶ�תΪENU�ٶ�
xsens_pos_by_vel(:,1) = cumtrapz(xsens_sample_time_fine,xsens_vel_data(:,1));
xsens_pos_by_vel(:,2) = cumtrapz(xsens_sample_time_fine,xsens_vel_data(:,2));
xsens_pos_by_vel(:,3) = cumtrapz(xsens_sample_time_fine,xsens_vel_data(:,3));

%%%%%%%%%%%%%%%%%%%%% �߾���RTK���ݵ���ȡ %%%%%%%%%%%%%%%%%%%%%
rtk_name = 'RTK\3.rtk';
% rtk��ʽ��
% ע�⣺��ת��Ϊ�Ƕ�
% "%.2f"%time+' '+"%.7f"%lat+' '+"%.7f"%lon+' '+"%.2f"%alt+' '
%         	+"%.3f"%yaw+' '+"%.3f"%pitch+' '+"%.3f"%roll+' '
%         	+"%.3f"%vE+' '+"%.3f"%vN+' '+"%.3f"%vU+'\n'
fileID = fopen([vehicular_file_path, rtk_name]);
rtk_data = textscan(fileID,'%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f');
rtk_time_tag = rtk_data{1};
rtk_lat = rtk_data{2};rtk_lon = rtk_data{3};rtk_alt = rtk_data{4};
rtk_yaw = rtk_data{5};rtk_pitch = rtk_data{6};rtk_roll = rtk_data{7};
rtk_vE = rtk_data{8};rtk_vN = rtk_data{9};rtk_vU = rtk_data{10};
rtk_velENU = [rtk_vE rtk_vN rtk_vU];
fclose(fileID);

%%%%%%%%%%%%%%%%%%%%%% laneto���ݵ���ȡ %%%%%%%%%%%%%%%%%%%%%
laneto_name = 'LineTo\3.lineTo';
% LaneTo��ʽ��
% str(time)+' '+'%.7f'%lon+' '+'%.7f'%lat+' '+'%.2f'%z+' '+
%             '%.2f'%heading+' '+'%.2f'%pitch+' '+'%.2f'%roll+'\n'
fileID=fopen([vehicular_file_path, laneto_name]);
laneto_data = textscan(fileID,'%f\t%f\t%f\t%f\t%f\t%f\t%f');    
laneto_time_tag = laneto_data{1};
laneto_lat = laneto_data{2};laneto_lon = laneto_data{3};
laneto_yaw = laneto_data{5};laneto_pitch = laneto_data{6};laneto_roll = laneto_data{7};
fclose(fileID);

%%%%%%%%%%%%%%%%%%%%%% single-gps���ݵ���ȡ %%%%%%%%%%%%%%%%%%%%%
single_gps_name = 'GPS\3.gps';
fileID=fopen([vehicular_file_path, single_gps_name]);
single_gps_data = textscan(fileID,'%f\t%f\t%f\t%f');    
single_gps_time_tag = single_gps_data{1};
single_gps_lat = single_gps_data{2};
single_gps_lon = single_gps_data{3};
single_gps_alt = single_gps_data{4};
fclose(fileID);

%% ʱ����봦��
GPS_LEAPSECOND = 16;       %2017��gpsʱ�����UTCʱ�����18s
UTC_start_time = datenum('2017-08-20 00:00:00','yyyy-mm-dd HH:MM:SS');
% xsens_start_time_raw = datenum('2017-08-22 12:47:03','yyyy-mm-dd HH:MM:SS');
xsens_start_time_raw = datenum('2017-08-22 12:32:50','yyyy-mm-dd HH:MM:SS');    %MT�϶�����ʱ����12:32:50
xsens_start_time = (xsens_start_time_raw-UTC_start_time)*24*3600;   %��ʱ��תΪ��UTC����00:00:00��������
% ������leap second
xsens_start_time = xsens_start_time + GPS_LEAPSECOND;
single_gps_time_tag = single_gps_time_tag + GPS_LEAPSECOND;
xsens_time_tag = (xsens_sample_time_fine + xsens_start_time);
disp(['xsens��rtk����ʼʱ�̲�(xsens-rtk):',num2str(xsens_start_time - rtk_time_tag(1)),'s']);
disp(['xsens��single-gps����ʼʱ�̲�(xsens-single_gps):',num2str(xsens_start_time - single_gps_time_tag(1)),'s']);


%% ��rtk��gps���ݽ��в���
rtk_acc(:,1) = diff(rtk_vE) * (1/dt);
rtk_acc(:,2) = diff(rtk_vN) * (1/dt);
rtk_acc(:,3) = diff(rtk_vU) * (1/dt);

single_gps_lat1 = single_gps_lat(1:end-1);
single_gps_lat2 = single_gps_lat(2:end);
single_gps_lon1 = single_gps_lon(1:end-1);
single_gps_lon2 = single_gps_lon(2:end);
[arclen,az] = distance(single_gps_lat1,single_gps_lon1,single_gps_lat2,single_gps_lon2,referenceEllipsoid('wgs84'));    
az = deg2rad(az);   %distance�����az�ǽǶ��ƣ�ת�ɻ�����
down_180_index = find(az<=180);
up_180_index = find(az>180);
single_gps_velENU = zeros(length(single_gps_lat1),3);
single_gps_velENU(down_180_index,1) = arclen(down_180_index).*sin(az(down_180_index)); %vel_E�Ĳ����ٶ�
single_gps_velENU(down_180_index,2) = arclen(down_180_index).*cos(az(down_180_index)); %vel_N�Ĳ����ٶ�
single_gps_velENU(up_180_index,1) = arclen(up_180_index).*(-sin(az(up_180_index)-deg2rad(180))); %vel_E�Ĳ����ٶ�
single_gps_velENU(up_180_index,2) = arclen(up_180_index).*(-cos(az(up_180_index)-deg2rad(180))); %vel_E�Ĳ����ٶ�


%% ���ݱ��棬�������������
ENU2NED = NED2ENU^-1;
%%%%%%%%%%%%%%% ����rtk������Ϊ�ο� %%%%%%%%%%%%%%%
% ����ΪNavego����ʽ���ٶ�ΪNED
ref_rtk.t = rtk_time_tag;
ref_rtk.lat = deg2rad(rtk_lat);     % Navego��Ҫ������ʽ��
ref_rtk.lon = deg2rad(rtk_lon);
ref_rtk.h = rtk_alt;
ref_rtk.vel = (ENU2NED*rtk_velENU')';   % Navego�е��ٶ�ΪNED��ʽ��
% ref_rtk.roll = deg2rad(rtk_roll);
% ref_rtk.pitch = deg2rad(rtk_pitch);
% ref_rtk.yaw = deg2rad(rtk_yaw);
temp_rtk_att = (ENU2NED*(deg2rad([rtk_roll rtk_pitch rtk_yaw]))')';
ref_rtk.roll = temp_rtk_att(:,1);
ref_rtk.pitch = temp_rtk_att(:,2);
ref_rtk.yaw = temp_rtk_att(:,3);
ref_rtk.kn = length(rtk_time_tag);
ref_rtk.DCMnb = zeros(length(rtk_yaw),9);
for i=1:length(rtk_yaw)
    temp = angle2dcm(deg2rad(rtk_roll(i)),deg2rad(rtk_pitch(i)),deg2rad(rtk_yaw(i)),'YXZ'); % ���ﲻ̫ȷ����ת��˳���Ƿ���ȷ
    ref_rtk.DCMnb(i,:) = reshape(ENU2NED*temp,[1,9]);
end
ref_rtk.freq = 1/mean(diff(rtk_time_tag));                 % ����Ƶ�ʣ����ظ߾��ȹߵ���rtkʵ����100hz��
save('ref_rtk.mat','ref_rtk');
%%%%%%%%%%%%%%% �;���GPS���� %%%%%%%%%%%%%%%
% ����ΪNavego����ʽ���ٶ�ΪNED
limit_down = 1; limit_up = length(single_gps_velENU);    % ���ڵ;���gps��vel�ǲ��������ģ����Գ��Ⱥ�ԭ����һ��
KT2MS = 0.514444;   % knot to m/s
single_gps.t = single_gps_time_tag(limit_down:limit_up);
single_gps.lat = deg2rad(single_gps_lat(limit_down:limit_up));     % Navego��Ҫ������ʽ��
single_gps.lon = deg2rad(single_gps_lon(limit_down:limit_up));
single_gps.h = single_gps_alt(limit_down:limit_up);
single_gps.vel = (ENU2NED*single_gps_velENU')';     % navego���ٶȷ���ΪNED
single_gps.vel = single_gps.vel(limit_down:limit_up,:);
single_gps.stdm = [5, 5, 10];                 % ֱ���õ�Navego��demo���ݣ�GPS positions standard deviations [lat lon h] (meters)
single_gps.stdv = 0.1 * KT2MS .* ones(1,3);   % ֱ���õ�Navego��demo���ݣ�GPS velocities standard deviations [Vn Ve Vd] (meters/s)
single_gps.larm = zeros(3,1);                 %  ֱ���õ�Navego��demo���ݣ�GPS lever arm [X Y Z] (meters)
single_gps.freq = 1/mean(diff(single_gps_time_tag));
save('single_gps.mat','single_gps');
%%%%%%%%%%%%%%% XSNES���� %%%%%%%%%%%%%%%
% ����xsens����ĳ��򣬽�imuУ�ص���ϵ
% xsens_att_calib_start = 1;
% xsens_att_calib_end = 10;
% xsens_att_calib = means(xsens_att(xsens_att_calib_start:xsens_att_calib_end,:));
% xsens_att_calib = deg2rad(xsens_att_calib);
% xsens_att_calib = angle2dcm(-xsens_att_calib(1),-xsens_att_calib(2),-xsens_att_calib(3),'XYZ');     % 8-22�ղ����У�xsens��X����ų�ͷ
% xsens_imu.fb = (ENU2NED*xsens_att_calib*xsens_acc_data')';          % ��xsensУ�ء�����acc_gen�ļ������ݣ����ٶ����Ӧ��ΪNED�µ�ֵ
xsens_imu.t = xsens_time_tag;
xsens_imu.fb = (ENU2NED*xsens_acc_data')';          % ����acc_gen�ļ������ݣ����ٶ����Ӧ��ΪNED�µ�ֵ
xsens_imu.fb(:,3) = - xsens_imu.fb(:,3);            % ����xsens��ֹʱZ�����еļ��ٶ�Ϊ+g��Ϊ�˺�Navego��ϣ��ĳ�-g
xsens_imu.wb = xsens_gyro_data;                     % �����ǲ���Ҫ��ת������Ϊ�����ʱ���õľ���bϵ�µ�
save('xsens_imu.mat','xsens_imu');

%% plot data
figure;
subplot(211);
plot(rtk_time_tag,rtk_vE,'.');
hold on;
plot(xsens_time_tag,xsens_vel_data(:,1),'.');
plot(single_gps_time_tag(1:end-1),single_gps_velENU(:,1),'.:');
title('x-axis vel');legend('rtk','xsensVel','singleGps');
subplot(212);
plot(rtk_time_tag,rtk_vN,'.');
hold on;
plot(xsens_time_tag,xsens_vel_data(:,2),'.');
plot(single_gps_time_tag(1:end-1),single_gps_velENU(:,2),'.:');
title('y-axis vel');legend('rtk','xsensVel','singleGps');

%%%%%%%% ������̬���� %%%%%%%%]
% xsens_att_calib_start = 1;
% xsens_att_calib_end = 10;
% xsens_att_calib = mean(xsens_att(xsens_att_calib_start:xsens_att_calib_end,:));
% xsens_att_calib = deg2rad(xsens_att_calib);
% xsens_att_calib = angle2dcm(-xsens_att_calib(1),-xsens_att_calib(2),-xsens_att_calib(3),'XYZ');     % 8-22�ղ����У�xsens��X����ų�ͷ
% temp_xsens_att = xsens_att;
% temp_xsens_att = (xsens_att_calib*temp_xsens_att')';
figure;
subplot(311);
plot(rtk_time_tag,rtk_yaw,'.');
hold on;
plot(xsens_time_tag,-xsens_att(:,3),'.');plot(laneto_time_tag,laneto_yaw,'.');
title('yaw');legend('rtk','xsens','laneto');
subplot(312);
plot(rtk_time_tag,rtk_pitch,'.');
hold on;
plot(xsens_time_tag,-xsens_att(:,2),'.');plot(laneto_time_tag,laneto_pitch,'.');
title('pitch');legend('rtk','xsens','laneto');
subplot(313);
plot(rtk_time_tag,rtk_roll,'.');
hold on;
plot(xsens_time_tag,-xsens_att(:,1),'.');plot(laneto_time_tag,laneto_roll,'.');
title('roll');legend('rtk','xsens','laneto');


figure;
subplot(211);
plot(rtk_time_tag(1:end-1),rtk_acc(:,1)); 
hold on;
plot(xsens_time_tag,xsens_free_acc_data(:,1));
plot(xsens_time_tag,xsens_acc_data(:,1));
title('x-axis free-acc');legend('rtk','xsens free','xsens raw');
subplot(212);
plot(rtk_time_tag(1:end-1),rtk_acc(:,2)); 
hold on;
plot(xsens_time_tag,xsens_free_acc_data(:,2));
plot(xsens_time_tag,xsens_acc_data(:,2));
title('y-axis free-acc');legend('rtk','xsens free','xsens raw');

figure;
subplot(211);
plot(single_gps_lat,single_gps_lon); 
hold on;
plot(xsens_lat,xsens_lon);
% plot(xsens_pos_by_vel(:,1),xsens_pos_by_vel(:,2));
title('location');legend('single-gps','xsens+gps');

%%%%%%%% ���ƴ��͸�Navego ������ %%%%%%%%
figure;
subplot(211);
plot(ref_rtk.t,ref_rtk.vel(:,1));
hold on;
plot(single_gps.t, single_gps.vel(:,1));
title('to Navego north-speed');legend('rtk','single_gps');
subplot(212);
plot(ref_rtk.t,ref_rtk.vel(:,2));
hold on;
plot(single_gps.t, single_gps.vel(:,2));
title('to Navego east-speed');legend('rtk','single_gps');

figure;
subplot(211);
plot(ref_rtk.t,ref_rtk.lat);
hold on;
plot(single_gps.t, single_gps.lat);
title('to Navego lat');legend('rtk','single_gps');
subplot(212);
plot(ref_rtk.t,ref_rtk.lon);
hold on;
plot(single_gps.t, single_gps.lon);
title('to Navego lon');legend('rtk','single_gps');

figure;
subplot(311);
plot(ref_rtk.t, ref_rtk.roll);
title('to Navego roll');legend('rtk');
subplot(312);
plot(ref_rtk.t, ref_rtk.pitch);
title('to Navego pitch');legend('rtk');
subplot(313);
plot(ref_rtk.t, ref_rtk.yaw);
hold on;
plot(single_gps.t,-az,'.');
title('to Navego yaw');legend('rtk','single gps');




