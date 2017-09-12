% SINS/GPS intergrated navigation simulation unsing kalman filter.
% Please run 'test_SINS_trj.m' to generate 'trj10ms.mat' beforehand!!!
% See also  test_SINS_trj, test_SINS, test_SINS_GPS_186, test_SINS_GPS_193.
% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 17/06/2011
glvs%这里初始化了一个全局变量，应该是用来进行参数传递的
psinstypedef(153);%决定状态参数的数量，这里就是15个的意思
trj = trjfile('trj10ms.mat');
% initial settings
[nn, ts, nts] = nnts(4, trj.ts);%nn是下采样数量，这里就是4，ts是时间间隔
imuerr = imuerrset(0.03, 100, 0.001, 5);%这句话用来初始化IMu的误差
%imuerr = imuerrset(eb, db, web, wdb, sqrtR0G, TauG, sqrtR0A, TauA, dKGii, dKAii, dKGij, dKAij)
%这里面的四个输入对应的就是陀螺仪的零偏，加速度计的零偏，以及对应的角度随机游走
imu = imuadderr(trj.imu, imuerr);
davp0 = avpseterr([30;-30;20], 0.1, [1;1;3]);%前面3个是角度偏差，中间的是速度，最后的是位置
ins = insinit(avpadderr(trj.avp0,davp0), ts); 
% KF filter
kf = kfinit(nts, davp0, imuerr);
len = length(imu); [avp, xkpk] = prealloc(fix(len*ts), 10, 2*kf.n+1);
% timebar(nn, len, '15-state SINS/GPS Simulation.'); 
ki = 1;
tbstep = floor(len/nn/100); tbi = timebar(1, 99, 'SINS/GPS Simulation.');
profile on
for k=1:nn:len-nn+1
    k1 = k+nn-1;  
    wvm = imu(k:k1,1:6);  t = imu(k1,end);
    ins = insupdate(ins, wvm);
    kf.Phikk_1 = kffk(ins);
    kf = kfupdate(kf);
    if mod(t,1)==0
        posGPS = trj.avp(k1,7:9)' + davp0(7:9).*randn(3,1);  % GPS pos simulation with some white noise
        kf = kfupdate(kf, ins.pos-posGPS, 'M');
        [kf, ins] = kffeedback(kf, ins, 1, 'avped');
        avp(ki,:) = [ins.avp', t];
        xkpk(ki,:) = [kf.xk; diag(kf.Pxk); t]';  ki = ki+1;
    end
%     timebar;
    if mod(tbi,tbstep)==0, timebar; end;  tbi = tbi+1;
end
profile viewer
% show results
avperr = avpcmp(avp, trj.avp);
insplot(avp);
inserrplot(avperr);
kfplot(xkpk, avperr, imuerr);

