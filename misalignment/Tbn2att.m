function att = Tbn2att(strapdown)
    T = strapdown;
    phi = atan(T(1,2)/T(2,2));
    if T(2,2)<0
        if T(1,2)>0
            phi = phi+pi;
        else
            phi = phi-pi;
        end
    end
    if phi<0;
        phi = phi+2*pi;
    end
    if phi>(2*pi)
        phi = phi-2*pi;
    end
%     phi = atan2(T(1,2),T(2,2));
    theta = asin(T(3,2));%-1.5266;
%     if theta<(-pi/2)
%         theta = theta + pi;
%     end
%     if theta >(pi/2)
%         theta = theta -pi;
%     end
    gama = atan(-T(3,1)/T(3,3));
    att = [theta;gama;phi];
end