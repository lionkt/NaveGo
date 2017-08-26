function [att_output] = Tbn2Attitude(strapdown)
    theta = asin(strapdown(2,3));
    gama = atan(-strapdown(1,3)/strapdown(3,3));
    if strapdown(3,3)<0
        if gama>0
            gama = gama - pi;
        else
            gama = gama + pi;
        end
    end
    phi = atan(strapdown(2,1)/strapdown(2,2));
    if abs(strapdown(2,2))<1e-10
        if strapdown(2,1)>0
            phi = pi/2;
        else
            phi = -pi/2;
        end
    elseif strapdown(2,2)<0
        if strapdown(2,1)>0
            phi = phi+pi;
        else
            phi = phi-pi;
        end
    else
        phi = phi;
    end
    if strapdown(2,2)<0
        phi = phi + pi;
    end
    if strapdown(2,2)>=0
        if phi <0
            phi = phi + 2*pi;
        end
    end
    att_output(1) = theta;
    att_output(2) = gama;
    att_output(3) = phi;
% 
% yaw= atan(-Cnb(1,2)/Cnb(2,2));
% if yaw<0&Cnb(2,2)>0
%     yaw=yaw+2*pi;
% elseif Cnb(2,2)<0
%     yaw=yaw+pi;
% end
% roll= atan(-Cnb(3,1)/Cnb(3,3));
% if roll<0&Cnb(3,3)<0
%     roll=roll+pi;
% elseif roll>0&Cnb(3,3)<0
%     roll=roll-pi;
% end
% pitch=atan(Cnb(3,2)/((Cnb(1,2)^2+Cnb(2,2)^2)^(1/2))); 
% att = [ pitch; 
%             roll;
%             yaw ]; 
%     theta=pitch;
%     gama=roll;
%     phi=yaw;
end