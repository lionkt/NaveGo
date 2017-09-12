%完整的组合导航算法
%%主要包含以下内容
%1、基本的组合导航滤波算法
%2、IMU和GPS数据产生方法
%3、反馈修正方法
%4、自适应滤波方法
%%原理介绍可以参见 readme.doc
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
len = size(imu,1); 
[avp, xkpk] = prealloc(fix(len*ts), 10, 2*kf.n+1);
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
    %GPS信号发生器
    %这里使用的是比较简单的白噪声
        posGPS = trj.avp(k1,7:9)' + davp0(7:9).*randn(3,1);  % GPS pos simulation with some white noise
    %%更复杂的GPS信号发生器%%
    % GPS data structure:
%         t: Mx1 time vector (seconds).
%       lat: Mx1 latitude (radians).
%       lon: Mx1 longitude (radians).
%         h: Mx1 altitude (m).
%       vel: Mx3 NED velocities (m/s).
%       std: 1x3 position standard deviations (rad, rad, m).
%      stdm: 1x3 position standard deviations (m, m, m).
%      stdv: 1x3 velocity standard deviations (m/s).
%      larm: 3x1 lever arm (x-right, y-fwd, z-down) (m).
%      freq: 1x1 sampling frequency (Hz).
%%
%     gps.stdm = [5, 5, 10];                 % GPS positions standard deviations [lat lon h] (meters)
%     gps.stdv = 0.1 * KT2MS .* ones(1,3);   % GPS velocities standard deviations [Vn Ve Vd] (meters/s)
%     gps.larm = zeros(3,1);                 % GPS lever arm [X Y Z] (meters)
%     gps.freq = 5;                          % GPS operation frequency (Hz)
%     gps = gps_err_profile(ref.lat(1), ref.h(1), gps); % Transform GPS manufacturer error units to SI units.   
%     [gps] = gps_gen(ref, gps);  % 这里的ref包括给定的时间，经纬高，速度信息
%     save gps.mat gps
        kf = kfupdate(kf, ins.pos-posGPS, 'M');
%反馈部分用于更新参数
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
% inserrplot(avperr);
% kfplot(xkpk, avperr, imuerr);
