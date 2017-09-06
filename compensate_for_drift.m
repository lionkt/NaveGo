function [lat_comp,lon_comp,velNED_comp] = compensate_for_drift(imu_e,index)
% 由于gps信号到来时，会使imu产生一个跳变。现在通过将跳变折算回上一段imu的解算结果中进行补偿
% imu_e：通过kalman filter处理过的imu数据
% index：gps信号到来时，imu数据的index
T = mean(imu_e.t(index(2:end))-imu_e.t(index(1:end-1)));    % 计算gps引起drift的周期
index_not_comp = index-1;   % gps没来之前的最后一个点的索引
index_comp = index;         % gps到来之后第一个点的索引
index_not_comp(index_not_comp==0) = [];
[arclen,az] = distance(imu_e.lat(index_not_comp),imu_e.lon(index_not_comp),imu_e.lat(index_comp),imu_e.lon(index_comp),referenceEllipsoid('wgs84'));   
az = deg2rad(az);   %distance输出的az是角度制，转成弧度制
pos_err_N = arclen.*cos(az);    %北向的误差
pos_err_E = arclen.*sin(az);    %东向的误差
acc_comp_N = 2*pos_err_N/(T^2);     % 需要补偿的N向加速度
acc_comp_E = 2*pos_err_E/(T^2);     % 需要补偿的E向加速度
acc_comp_N = acc_comp_N(:);
acc_comp_E = acc_comp_E(:);
%%%%%%%%%% 开始补偿 %%%%%%%%%%
velNED_comp(1:index_not_comp,:) = imu_e.vel(1:index_not_comp,:);
% 但是发现这种补偿应该放在组合导航里面去做

end

