function [lat_comp,lon_comp,velNED_comp] = compensate_for_drift(imu_e,index)
% ����gps�źŵ���ʱ����ʹimu����һ�����䡣����ͨ���������������һ��imu�Ľ������н��в���
% imu_e��ͨ��kalman filter�������imu����
% index��gps�źŵ���ʱ��imu���ݵ�index
T = mean(imu_e.t(index(2:end))-imu_e.t(index(1:end-1)));    % ����gps����drift������
index_not_comp = index-1;   % gpsû��֮ǰ�����һ���������
index_comp = index;         % gps����֮���һ���������
index_not_comp(index_not_comp==0) = [];
[arclen,az] = distance(imu_e.lat(index_not_comp),imu_e.lon(index_not_comp),imu_e.lat(index_comp),imu_e.lon(index_comp),referenceEllipsoid('wgs84'));   
az = deg2rad(az);   %distance�����az�ǽǶ��ƣ�ת�ɻ�����
pos_err_N = arclen.*cos(az);    %��������
pos_err_E = arclen.*sin(az);    %��������
acc_comp_N = 2*pos_err_N/(T^2);     % ��Ҫ������N����ٶ�
acc_comp_E = 2*pos_err_E/(T^2);     % ��Ҫ������E����ٶ�
acc_comp_N = acc_comp_N(:);
acc_comp_E = acc_comp_E(:);
%%%%%%%%%% ��ʼ���� %%%%%%%%%%
velNED_comp(1:index_not_comp,:) = imu_e.vel(1:index_not_comp,:);
% ���Ƿ������ֲ���Ӧ�÷�����ϵ�������ȥ��

end

