function Cnb = a2Tbn(att)
% Convert Euler angles to direction cosine matrix(DCM).
%
% Prototype: Cnb = a2mat(att)
% Input: att - att=[pitch; roll; yaw] in radians
% Output: Cnb - DCM from body-frame to navigation-frame
 s = sin(att); c = cos(att);
    si = s(1); sj = s(2); sk = s(3); 
    ci = c(1); cj = c(2); ck = c(3);
    Cnb = [ cj*ck-si*sj*sk, -ci*sk,  sj*ck+si*cj*sk;
            cj*sk+si*sj*ck,  ci*ck,  sj*sk-si*cj*ck;
           -ci*sj,           si,     ci*cj           ];
end

