function result = findPitchRoll(t,dict,offset_ms)

result= zeros(size(t,1),3);
t=t + offset_ms;
tIndex = 1;

for i=1:size(t,1)
    for j = tIndex:size(dict,1)
        if( (dict(j,1) - t(i) ) < -5)
            continue;
        elseif ( (dict(j,1)- t(i))> 0)
            result(i,:) = dict(j,4:6);
            tIndex = j - 1;
            break;
        else
            result(i,:) = dict(j,4:6);
            tIndex = j;
            break;
        end
    end
    
end