clear all;
%用于仿真纯惯性导航
trj=load('trj10ms.mat');
trj=trj.trj.imu;
glvs
avp00=[-0.000145232358978132;-0.000145655439968606;-0.00290889267385456;0.100000000000000;0.100000000000000;0.100000000000000;0.597707861251962;1.90083379189668;390];
trj(:,6)=trj(:,6);
avp = inspure(trj, avp00,'V');
n=size(avp,1);
figure;
plot3(avp(:,7),avp(:,8),avp(:,9))
hold on
plot3(avp(1,7),avp(1,8),avp(1,9),'ro')



