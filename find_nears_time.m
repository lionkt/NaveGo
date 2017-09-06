function [ index ] = find_nears_time( imu_time_tag, gps_time_tag )
%   寻找imu和gps时间戳最接近的imu时间的下标
if (imu_time_tag(end)<=gps_time_tag(1))||(imu_time_tag(1)>=gps_time_tag(end))
    disp('error: imu时间和gps没有对应的部分');
    return;
end
i = 1;
index = [];
disp('开始时间对齐');
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

disp('时间对齐结束');
end

