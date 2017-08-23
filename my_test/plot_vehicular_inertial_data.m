file_path = 'F:\momenta文件夹\2017-8-11跑车数据\20170811车载设备数据\3\';
good_imu_name = '2017-08-11-20-48-57_imu';
rtk_name = '2017-08-11-20-48-57_rtk';
laneto_name = 'LaneTo3';
PROCESS_GOOD_IMU = true;
PROCESS_RTK = true;
PROCESS_LANETO = true;
if PROCESS_GOOD_IMU == true
    % 高精度imu格式
    % "%.2f"%time+' '+"%.4f"%aX+' '+"%.4f"%aY+' '+"%.4f"%aZ+' '
    %         	+"%.4f"%vX+' '+"%.4f"%vY+' '+"%.4f"%vZ+'\n'
    fileID=fopen([file_path, good_imu_name,'.txt']);
    good_imu_data = textscan(fileID,'%f\t%f\t%f\t%f\t%f\t%f\t%f');
    fclose(fileID);
end
if PROCESS_RTK == true
    % rtk格式：
    % "%.2f"%time+' '+"%.7f"%lat+' '+"%.7f"%lon+' '+"%.2f"%alt+' '
    %         	+"%.3f"%yaw+' '+"%.3f"%pitch+' '+"%.3f"%roll+' '
    %         	+"%.3f"%vE+' '+"%.3f"%vN+' '+"%.3f"%vU+'\n'
    fileID=fopen([file_path, rtk_name,'.txt']);
    rtk_data = textscan(fileID,'%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f');
    rtk_time_stamp = rtk_data{1};
    rtk_lat = rtk_data{2};
    rtk_lon = rtk_data{3};
    fclose(fileID);
end
if PROCESS_LANETO == true
    % LaneTo格式：
    % str(time)+' '+'%.7f'%lon+' '+'%.7f'%lat+' '+'%.2f'%z+' '+
    %             '%.2f'%heading+' '+'%.2f'%pitch+' '+'%.2f'%roll+'\n'
    fileID=fopen([file_path, laneto_name,'.txt']);
    laneto_data = textscan(fileID,'%f\t%f\t%f\t%f\t%f\t%f\t%f');    
    laneto_time_stamp = laneto_data{1};
    laneto_lon = laneto_data{2};
    laneto_lat = laneto_data{3};
    fclose(fileID);
end

%% plot data
figure;
plot(rtk_lon,rtk_lat,'.');
hold on; grid on;
plot(laneto_lon,laneto_lat,'.');










