clc;
clear all;
%% ������ȡ
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

XSENS_SAMPLE_FREQUENCY = 100;     % xsens����Ƶ��100hz
dt = 1/XSENS_SAMPLE_FREQUENCY;
xsens_sample_time_fine = (xsens_sample_time_fine-xsens_sample_time_fine(1))/10000;        %xsensʱ��ת������Ϊ10000 = 1s

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
laneto_lat = laneto_data{2};laneto_lon = laneto_data{3};laneto_alt = laneto_data{4};
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

%%%%%%%%%%%%%%%%%%%%%% �߾��ȹߵ����ݵ���ȡ %%%%%%%%%%%%%%%%%%%%%
good_imu_name = 'RTK\3.imu';
fileID=fopen([vehicular_file_path, good_imu_name]);
good_imu_data = textscan(fileID,'%f\t%f\t%f\t%f\t%f\t%f\t%f');    
good_imu_time_tag = good_imu_data{1};
good_imu_gyroX = good_imu_data{2};      % �߾��ȹߵ������������Ϊ rad/s
good_imu_gyroY = good_imu_data{3};
good_imu_gyroZ = good_imu_data{4};
good_imu_accX = good_imu_data{5};
good_imu_accY = good_imu_data{6};
good_imu_accZ = good_imu_data{7};
fclose(fileID);

%% ���ݴ洢����ֵ��������ֶ�
single_gps_data_mode = 0; % 0-ʹ��single_gps������Ϊsingle_gps��1-ʹ��rtk�����ġ���١���single_gps��2-ʹ��laneto������Ϊsingle_gps��3-ʹ��xsens�Դ���gps�����λ��
abstract_some_gps_data_flag = false;    % �޳����ֵ;���gps���ݣ����Զ�̬�궨��Ч��
use_interploation_to_single_gps = false; % �Ե;���gps���в�ֵ������ߵ;���gps������Ƶ��
use_single_gps_Z_vel = false;           % ���;���gps��z���ٶ�����kalman��Ŀǰ���Խ��������󷴶����ã�


%% ʱ����봦��
XSENS_GPS_LEAPSECOND = 15;       %xsens��leapsecond������2017��gpsʱ�����UTCʱ�����18s�����Ƿ���ȡ15s����ʱ�ٶȲ��ܶ���
single_gps_leapsecond = 15;      %�;���gps��leapsecond���������Է�����15sʱ8-22�ĵ;���gps��rtk���ݶԵ����
UTC_start_time = datenum('2017-08-20 00:00:00','yyyy-mm-dd HH:MM:SS');
% xsens_start_time_raw = datenum('2017-08-22 12:47:03','yyyy-mm-dd HH:MM:SS');
xsens_start_time_raw = datenum('2017-08-22 12:32:50','yyyy-mm-dd HH:MM:SS');    %MT�϶�����ʱ����12:32:50
xsens_start_time = (xsens_start_time_raw-UTC_start_time)*24*3600;   %��ʱ��תΪ��UTC����00:00:00��������
% ������leap second
xsens_start_time = xsens_start_time + XSENS_GPS_LEAPSECOND;
single_gps_time_tag = single_gps_time_tag + single_gps_leapsecond;
xsens_time_tag = (xsens_sample_time_fine + xsens_start_time);
disp(['xsens��rtk����ʼʱ�̲�(xsens-rtk):',num2str(xsens_start_time - rtk_time_tag(1)),'s']);
disp(['xsens��single-gps����ʼʱ�̲�(xsens-single_gps):',num2str(xsens_start_time - single_gps_time_tag(1)),'s']);


%% ��rtk��gps���ݽ��д���
rtk_acc(:,1) = diff(rtk_vE) * (1/dt);
rtk_acc(:,2) = diff(rtk_vN) * (1/dt);
rtk_acc(:,3) = diff(rtk_vU) * (1/dt);
%%%%%%%% single gps��ֵ������������������������Ƶ�� %%%%%%%%
if use_interploation_to_single_gps == true
    single_gps_interp_step = 1/2;   %��ֵ��ʱ����ļ��
    disp('====== ��ʼ�;���GPS��ֵ�������������� ======');
    single_gps_lat = interp1(single_gps_time_tag, single_gps_lat, [single_gps_time_tag(1):single_gps_interp_step:single_gps_time_tag(end)],'spline');
    single_gps_lon = interp1(single_gps_time_tag, single_gps_lon, [single_gps_time_tag(1):single_gps_interp_step:single_gps_time_tag(end)],'spline');
    single_gps_alt = interp1(single_gps_time_tag, single_gps_alt, [single_gps_time_tag(1):single_gps_interp_step:single_gps_time_tag(end)],'spline');
    single_gps_time_tag = [single_gps_time_tag(1):single_gps_interp_step:single_gps_time_tag(end)];
    single_gps_lat = single_gps_lat(:); 
    single_gps_lon = single_gps_lon(:);
    single_gps_alt = single_gps_alt(:);
    single_gps_time_tag = single_gps_time_tag(:);
else
    disp('====== �;���gps����ֵ ======');
    single_gps_interp_step = 1/1;   %ԭʼgps���ݵ�ʱ����
end

%%%%%%%% single gps��ּ����ٶ� %%%%%%%%
single_gps_lat1 = single_gps_lat(1:end-1);
single_gps_lat2 = single_gps_lat(2:end);
single_gps_lon1 = single_gps_lon(1:end-1);
single_gps_lon2 = single_gps_lon(2:end);
[arclen,az] = distance(single_gps_lat1,single_gps_lon1,single_gps_lat2,single_gps_lon2,referenceEllipsoid('wgs84'));    
az = deg2rad(az);   %distance�����az�ǽǶ��ƣ�ת�ɻ�����
single_gps_velENU = zeros(length(single_gps_lat1),3);
single_gps_velENU(:,1) = arclen.*sin(az)/single_gps_interp_step; %vel_E���ٶ�
single_gps_velENU(:,2) = arclen.*cos(az)/single_gps_interp_step; %vel_N���ٶ�
if use_single_gps_Z_vel == true
    disp('=== �;���gps����gpsZ���ٶ�Լ�� ===');
    single_gps_velENU(:,3) = (single_gps_alt(2:end)-single_gps_alt(1:end-1))/single_gps_interp_step; %vel_U���ٶ�
else
    disp('=== �;���gps������gpsZ���ٶ�Լ�� ===');
end
%%%%%%%% laneto��ּ����ٶ� %%%%%%%%
laneto_lat1 = laneto_lat(1:end-1);
laneto_lat2 = laneto_lat(2:end);
laneto_lon1 = laneto_lon(1:end-1);
laneto_lon2 = laneto_lon(2:end);
[arclen2,az2] = distance(laneto_lat1,laneto_lon1,laneto_lat2,laneto_lon2,referenceEllipsoid('wgs84'));    
az2 = deg2rad(az2);   %distance�����az�ǽǶ��ƣ�ת�ɻ�����
laneto_velENU = zeros(length(laneto_lat1),3);
laneto_velENU(:,1) = arclen2.*sin(az2)/1; %vel_E���ٶ�
laneto_velENU(:,2) = arclen2.*cos(az2)/1; %vel_N���ٶ�
if use_single_gps_Z_vel == true
    disp('=== laneto����gpsZ���ٶ�Լ�� ===');
    laneto_velENU(:,3) = (laneto_alt(2:end)-laneto_alt(1:end-1))/1; %vel_U���ٶ�
else
    disp('=== laneto������gpsZ���ٶ�Լ�� ===');
end

%% ���ݱ��棬�������������
ENU2NED = NED2ENU^-1;
%%%%%%%%%%%%%%% ����rtk������Ϊ�ο� %%%%%%%%%%%%%%%
% ����ΪNavego����ʽ���ٶ�ΪNED
ref_rtk.t = rtk_time_tag;
ref_rtk.lat = deg2rad(rtk_lat);     % Navego��Ҫ������ʽ��
ref_rtk.lon = deg2rad(rtk_lon);
ref_rtk.h = rtk_alt;
ref_rtk.vel = (ENU2NED*rtk_velENU')';   % Navego�е��ٶ�ΪNED��ʽ��
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
if single_gps_data_mode == 0
    disp('==== ʹ�õ;���gps���� ====');
    %%%% ���;��ȵ�gps����ΪNavego����ʽ���ٶ�ΪNED
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
    if abstract_some_gps_data_flag == true
        %%%% ����single gps��һ�����ݣ�����xsensƯ�Ƶĳ̶�
        disp('==== �޳����ֵ;���gps���� ====');
        start_abstract_index = round(6.8*length(single_gps_velENU)/10);
        end_abstract_index = round(7.0*length(single_gps_velENU)/10);
        single_gps.t(start_abstract_index:end_abstract_index) = [];
        single_gps.lat(start_abstract_index:end_abstract_index) = [];
        single_gps.lon(start_abstract_index:end_abstract_index) = [];
        single_gps.h(start_abstract_index:end_abstract_index) = [];
        single_gps.vel(start_abstract_index:end_abstract_index,:) = [];
    end
    save('single_gps.mat','single_gps');

elseif single_gps_data_mode == 1
    %%%% ��rtk������α��;���gps�����ԽǶȽ���
    disp('==== ʹ��rtk����α��;���gps ====');
    RTK_SAMPLE_FERQUENCY = 100;
    KT2MS = 0.514444;   % knot to m/s
    single_gps.t = ref_rtk.t(1:RTK_SAMPLE_FERQUENCY:end);
    single_gps.lat = ref_rtk.lat(1:RTK_SAMPLE_FERQUENCY:end);
    single_gps.lon = ref_rtk.lon(1:RTK_SAMPLE_FERQUENCY:end);
    single_gps.h = ref_rtk.h(1:RTK_SAMPLE_FERQUENCY:end);
    single_gps.vel = ref_rtk.vel(1:RTK_SAMPLE_FERQUENCY:end,:);
    single_gps.stdm = [5, 5, 10];                 % ֱ���õ�Navego��demo���ݣ�GPS positions standard deviations [lat lon h] (meters)
    single_gps.stdv = 0.1 * KT2MS .* ones(1,3);   % ֱ���õ�Navego��demo���ݣ�GPS velocities standard deviations [Vn Ve Vd] (meters/s)
    single_gps.larm = zeros(3,1);                 %  ֱ���õ�Navego��demo���ݣ�GPS lever arm [X Y Z] (meters)
    single_gps.freq = 1/mean(diff(single_gps_time_tag));    
    if abstract_some_gps_data_flag == true    
        % ��ʼ�޳�����gps����
        disp('==== �޳����ֵ;���gps���� ====');
        start_abstract_index = round(8.8*length(single_gps.t)/10);
        end_abstract_index = round(9.0*length(single_gps.t)/10);
        single_gps.t(start_abstract_index:end_abstract_index) = [];
        single_gps.lat(start_abstract_index:end_abstract_index) = [];
        single_gps.lon(start_abstract_index:end_abstract_index) = [];
        single_gps.h(start_abstract_index:end_abstract_index) = [];
        single_gps.vel(start_abstract_index:end_abstract_index,:) = [];
    end
    save('single_gps.mat','single_gps');
    
elseif single_gps_data_mode == 2
    %%%% ��laneto����������gps���ٶ�ΪNED
    disp('==== ʹ��laneto����α��;���gps ====');
    limit_down = 1; limit_up = length(laneto_velENU);    % ���ڵ;���gps��vel�ǲ��������ģ����Գ��Ⱥ�ԭ����һ��
    KT2MS = 0.514444;   % knot to m/s
    single_gps.t = laneto_time_tag(limit_down:limit_up);
    single_gps.lat = deg2rad(laneto_lat(limit_down:limit_up));     % Navego��Ҫ������ʽ��
    single_gps.lon = deg2rad(laneto_lon(limit_down:limit_up));
    single_gps.h = laneto_alt(limit_down:limit_up);
    single_gps.vel = (ENU2NED*laneto_velENU')';     % navego���ٶȷ���ΪNED
    single_gps.vel = single_gps.vel(limit_down:limit_up,:);
    single_gps.stdm = [5, 5, 10];                 % ֱ���õ�Navego��demo���ݣ�GPS positions standard deviations [lat lon h] (meters)
    single_gps.stdv = 0.1 * KT2MS .* ones(1,3);   % ֱ���õ�Navego��demo���ݣ�GPS velocities standard deviations [Vn Ve Vd] (meters/s)
    single_gps.larm = zeros(3,1);                 %  ֱ���õ�Navego��demo���ݣ�GPS lever arm [X Y Z] (meters)
    single_gps.freq = 1/mean(diff(single_gps_time_tag));
    save('single_gps.mat','single_gps');
else
    disp('single_gps����ģʽѡ�����');
end

%%%%%%%%%%%%%%% XSNES���� %%%%%%%%%%%%%%%
% ����xsens����ĳ��򣬽�imuУ�ص���ϵ
% xsens_att_calib_start = 1;
% xsens_att_calib_end = 10;
% xsens_att_calib = means(xsens_att(xsens_att_calib_start:xsens_att_calib_end,:));
% xsens_att_calib = deg2rad(xsens_att_calib);
% xsens_att_calib = angle2dcm(-xsens_att_calib(1),-xsens_att_calib(2),-xsens_att_calib(3),'XYZ');     % 8-22�ղ����У�xsens��X����ų�ͷ
% xsens_imu.fb = (ENU2NED*xsens_att_calib*xsens_acc_data')';          % ��xsensУ�ء�����acc_gen�ļ������ݣ����ٶ����Ӧ��ΪNED�µ�ֵ
xsens_imu.t = xsens_time_tag;
% xsens_imu.fb = (ENU2NED*xsens_acc_data')';          
xsens_imu.fb = xsens_acc_data;
xsens_imu.fb(:,3) = xsens_imu.fb(:,3);            % ����xsens��ֹʱZ�����еļ��ٶ�Ϊ+g��Navego��Ҳ����+gΪ��׼
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

%%%%%%%% ������̬���� %%%%%%%%
% xsens_att_calib_start = 1;
% xsens_att_calib_end = 10;
% xsens_att_calib = mean(xsens_att(xsens_att_calib_start:xsens_att_calib_end,:));
% xsens_att_calib = deg2rad(xsens_att_calib);
% xsens_att_calib = angle2dcm(-xsens_att_calib(1),-xsens_att_calib(2),-xsens_att_calib(3),'XYZ');     % 8-22�ղ����У�xsens��X����ų�ͷ
% temp_xsens_att = xsens_att;
% temp_xsens_att = (xsens_att_calib*temp_xsens_att')';
imu_e_time_tag = load('../calculation_data/imu_e_time.mat');         % ��ȡNavego����õ���imu������ʱ���
imu_e_att = load('../calculation_data/imu_e_att.mat');              % ��ȡNavego����õ���imu��̬��
imu_e_time_tag = imu_e_time_tag.imu_e_time_for_save;
imu_e_att = imu_e_att.imu_e_att_for_save;
imu_e_att = rad2deg(imu_e_att);

figure;
subplot(311);
plot(rtk_time_tag,rtk_yaw,'.');
hold on;
plot(xsens_time_tag,-xsens_att(:,3),'.');plot(laneto_time_tag,laneto_yaw,'.');
plot(imu_e_time_tag,-imu_e_att(:,3),'.');         % Navego����õ���imu��̬��
ylabel('[deg]');title('yaw');legend('rtk','xsens','laneto','Navego-calculation');
subplot(312);
plot(rtk_time_tag,rtk_pitch,'.');
hold on;
plot(xsens_time_tag,-xsens_att(:,2),'.');plot(laneto_time_tag,laneto_pitch,'.');
plot(imu_e_time_tag,-imu_e_att(:,2),'.');         % Navego����õ���imu��̬��
ylabel('[deg]');title('pitch');legend('rtk','xsens','laneto','Navego-calculation');
subplot(313);
plot(rtk_time_tag,rtk_roll,'.');
hold on;
plot(xsens_time_tag,-xsens_att(:,1),'.');plot(laneto_time_tag,laneto_roll,'.');
plot(imu_e_time_tag,-imu_e_att(:,1),'.');         % Navego����õ���imu��̬��
ylabel('[deg]');title('roll');legend('rtk','xsens','laneto','Navego-calculation');

%%%%%%%% ���Ƽ��ٶ����� %%%%%%%%
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

%%%%%%%% ����λ������ %%%%%%%%
figure;
subplot(211);
plot(single_gps_lat,single_gps_lon); 
hold on;
plot(xsens_lat,xsens_lon);
% plot(xsens_pos_by_vel(:,1),xsens_pos_by_vel(:,2));
title('location');legend('single-gps','navego:xsens+gps');

%%%%%%%% ���ƴ��͸�Navego ������ %%%%%%%%
figure;
subplot(211);
plot(ref_rtk.t,ref_rtk.vel(:,1));
hold on;
plot(single_gps.t, single_gps.vel(:,1),'.');
title('to Navego north-speed');legend('rtk','single_gps');
subplot(212);
plot(ref_rtk.t,ref_rtk.vel(:,2));
hold on;
plot(single_gps.t, single_gps.vel(:,2),'.');
title('to Navego east-speed');legend('rtk','single_gps');

figure;
subplot(211);
plot(ref_rtk.t,ref_rtk.lat);
hold on;
plot(single_gps.t, single_gps.lat,'.');
title('to Navego lat');legend('rtk','single_gps');
subplot(212);
plot(ref_rtk.t,ref_rtk.lon);
hold on;
plot(single_gps.t, single_gps.lon,'.');
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




