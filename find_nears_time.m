function [ index ] = find_nears_time( imu_time_tag, gps_time_tag )
%   Ѱ��imu��gpsʱ�����ӽ���imuʱ����±�
if (imu_time_tag(end)<=gps_time_tag(1))||(imu_time_tag(1)>=gps_time_tag(end))
    disp('error: imuʱ���gpsû�ж�Ӧ�Ĳ���');
    return;
end
i = 1;
index = [];
disp('��ʼʱ�����');
for k=1:length(gps_time_tag)
    if(gps_time_tag(k)<imu_time_tag(1))
        continue;
    end
    if(i+1>length(imu_time_tag))
        break;
    end
    while(true)
        if(imu_time_tag(i)<=gps_time_tag(k))&&(imu_time_tag(i+1)>gps_time_tag(k))
            index = [index i+1];
            break;
        end
        if(i==length(imu_time_tag))
            break;
        end
        i = i+1;
    end
end

disp('ʱ��������');
end

