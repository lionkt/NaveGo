clc;
clear all;

number1=20000;     %迭代次数
% head=zeros(20000,1);  %初始化姿态
% pitch=zeros(20000,1);
% roll=zeros(20000,1);
% h = zeros(20000,1);
% Latitude = zeros(20000,1);
% Longitude = zeros(20000,1);
% Ve = zeros(20000,1);
% Vn = zeros(20000,1);
% Vu = zeros(20000,1);

pi = 3.1415926535;   %初始化变量
deg = 180/pi;
rad = pi/180;
dt = 1/400;


Omiga=0.00007272205216643040; %地球自转角速度
f=1.0/298.257;
Re=6378137.0;
num=0;
W_iee = [0;0;Omiga];    %地球自转角速度


% %姿态角
% pitch0 = 1.9113951 * rad;  %俯仰角
% head0 =  89.850004 * rad;  %偏航角
% roll0 =  1.0572407 * rad;  %横滚角

%位置，经纬度,高度
Longitude0 = 116.326680426 * rad;  %经度  弧度制
Latitude0 = 39.993771328 * rad;    %纬度
High0 = 41.81;
g0=9.80151;
h(1) = High0;
Latitude(1) = Latitude0;
Longitude(1) = Longitude0;
Rn=Re*(1+f*(sin(Latitude0))^2);
Rm=Re*(1-2*f+3*f*(sin(Latitude0))^2);
%速度
Ve0 = 0.0;
Vn0 = 0.0;
Vu0 = 0.0;
Ve(1) = 0.0; %速度解算输出
Vn(1) = 0.0;
Vu(1) = 0.0;

Cen(1,1)=-sin(Longitude0);   %初始化位置矩阵
Cen(1,2)=cos(Longitude0);
Cen(1,3)=0;
Cen(2,1)=-sin(Latitude0)*cos(Longitude0);
Cen(2,2)=-sin(Latitude0)*sin(Longitude0);
Cen(2,3)=cos(Latitude0);
Cen(3,1)=cos(Latitude0)*cos(Longitude0);
Cen(3,2)=cos(Latitude0)*sin(Longitude0);
Cen(3,3)=sin(Latitude0);

%初始对准
W_ibb = [0;0;0];         %读取初始W_ibb
data0=xlsread('D:\momenta文件夹\2017-8-11跑车数据\Mti-G-710标定后的数据\MT_2017-08-11-21h04-000_processed.csv');
% data0=xlsread('C:\Users\Sophia\Documents\XSENS\csv文件\MT_07701495-20170808-1200.csv');

% W_ibb(1)=mean(data0(:,2))*rad;
% W_ibb(2)=mean(data0(:,3))*rad;
% W_ibb(3)=mean(data0(:,4))*rad;
% fxb = mean(data0(:,5))*g0;
% fyb = mean(data0(:,6))*g0;
% fzb = - mean(data0(:,7))*g0;
% fb = [fxb;fyb;fzb];
% load fb.mat;
% fbdata = fb';
% load wbib.mat;
% wbib = wbib';
fbdata = data0(:,5:7)*g0;
wbib = data0(:,2:4)*rad;
DataLen = length(fbdata);
W_ibb(1) = mean(wbib(:,1));
W_ibb(2) = mean(wbib(:,2));
W_ibb(3) = mean(wbib(:,3));
fxb = mean(fbdata(:,1));
fyb = mean(fbdata(:,2));
fzb = mean(fbdata(:,3));
fb = [fxb;fyb;fzb];
%%%%%%%%%%%%%%粗对准%%%%%%%%%%%%%%
V_bT = [fxb                        fyb                        fzb;
        W_ibb(1)                   W_ibb(2)                   W_ibb(3);
        fyb*W_ibb(3)-fzb*W_ibb(2)  fzb*W_ibb(1)-fxb*W_ibb(3)  fxb*W_ibb(2)-fyb*W_ibb(1)];

V_nT = [0                        0                     -g0;
        0                        Omiga*cos(Latitude0) Omiga*sin(Latitude0);
        g0*Omiga*cos(Latitude0) 0                     0];



V_nT_inv = [ 0                  0                        (g0*Omiga*cos(Latitude0))^(-1);
            tan(Latitude0)/g0  1/(Omiga*cos(Latitude0))  0;
            -1/g0              0                         0];

Tbp=V_nT_inv*V_bT;    %实际是Tbp

%计算Tpn
fai = zeros(3,1);
fp = Tbp*fb;
W_ibp = Tbp*W_ibb;
fai(1) = -fp(2)/g0;
fai(2) = fp(1)/g0;
fai(3) = (W_ibp(1)+fai(2)*Omiga*sin(Latitude0))/(Omiga*cos(Latitude0));
Tnp = [1        fai(3)      -fai(2);
      -fai(3)   1           fai(1);
       fai(2)   -fai(1)     1     ];
Tpn = Tnp';
Tbn = Tpn*Tbp;
Tnb = Tbn';
Tbn = Tbn/norm(Tbn);
att0 =Tbn2Attitude(Tbn);
pitch0 = att0(1);
roll0 = att0(2);
head0 = att0(3);

%%%%%%%%%%%%%开始精对准%%%%%%%%%%%%%%
head(1) = head0;      %解算输出值
pitch(1) = pitch0;
roll(1) = roll0;

pos = [39.999613*pi/180,116.339999*pi/180,70];%纬度经度高度
vn = [0 0 0];
% eth=earth(pos,vn);

q = Tbn2Q(Tbn);

% W_ibbx=data0(:,2)*rad;
% W_ibby=data0(:,3)*rad;
% W_ibbz=data0(:,4)*rad;
% fbx = data0(:,5)*g0;
% fby = data0(:,6)*g0;
% fbz = - data0(:,7)*g0;

W_ibbx=wbib(:,1);
W_ibby=wbib(:,2);
W_ibbz=wbib(:,3);
fbx = fbdata(:,1);
fby = fbdata(:,2);
fbz = fbdata(:,3);

fb = [fbx fby fbz];
fn = Tbn*fb(1,:)';

W_enn = [0;0;0];
W_enn(1) = -Vn0/(Rm+High0);
W_enn(2) = Ve0/(Rn+High0);
W_enn(3) = Ve0*tan(Latitude0)/(Rn+High0);
W_ien = Cen*W_iee;
W_nbb_0=W_ibb-Tnb*(W_ien+W_enn);

dot_vn = fn + [0;0;g0];
w=2;
f=2;

for i=1:(length(wbib)-1)/2
     for m=1:2  
    %%更新速度
    %input wbib按列存
%     wbnb(:,m+1)=ib2nb(wbib(:,w),(Tnb)',eth);
    wbnb(:,m+1)=wbib(w,:)'-Tnb*(W_ien+W_enn);
    w=w+1;
    %input fb按列存
%     dot_vn(:,2)=fb2fn(fb(:,f),Tnb,eth);
    dot_vn(:,2) = Tbn*fb(f,:)' + [0;0;g0];
    f=f+1;
    vn=0.01*(dot_vn(:,1)+dot_vn(:,2))/2;
%     fprintf(fid1,'%f\t%f\t%f\n',vn(1),vn(2),vn(3)); 
    dot_vn(:,1)=dot_vn(:,2);
%   eth=earth(pos,vn);
    end
    %% (3)attitude updating
    k1 = 1/2*rq2m([0;wbnb(:,1)])*q;
    k2 = 1/2*rq2m([0;wbnb(:,2)])*(q+0.01*k1);
    k3 = 1/2*rq2m([0;wbnb(:,2)])*(q+0.01*k2);
    k4 = 1/2*rq2m([0;wbnb(:,3)])*(q+0.02*k3);
    q = q + 0.02/6*(k1+2*(k2+k3)+k4);
    %q=rgkt4(wbnb,q,0.02);
    Tbn=Q2Tbn(q);
    Tnb = Tbn';
    att(i,:)=Tbn2Attitude(Tbn); 
    %q=qnormlz(q);
    wbnb(:,1)=wbnb(:,3);
end


%%%%%Kalman Filter%%%%%%%%%
x=zeros(10,1);

t=0.01;
R=diag([(0.1)^2,(0.1)^2],0)/t;
H=[eye(2),zeros(2,8)];
h=[(5e-5*g0)^2,(5e-5*g0)^2,(0.01/3600*pi/180)^2,(0.01/3600*pi/180)^2,(0.01/3600*pi/180)^2,0,0,0,0,0];
%h=[0,0,0,0,0,(2/60)^2,(2/60)^2,(4.2/60*pi/180)^2,(4.2/60*pi/180)^2,(4.2/60*pi/180)^2];
%P=diag([0.1^2,0.1^2,(pi/180)^2,(pi/180)^2,(pi/180)^2,(2/60)^2,(2/60)^2,(4.2/60*pi/180)^2,(4.2/60*pi/180)^2,(4.2/60*pi/180)^2],0);
P=diag([0.1^2,0.1^2,(pi/180)^2,(pi/180)^2,(pi/180)^2,(1e-4*g0)^2,(1e-4*g0)^2,(0.02/3600*pi/180)^2,(0.02/3600*pi/180)^2,(0.02/3600*pi/180)^2],0);
Q=diag(h,0);
Z=[randn(1,DataLen)*10^-6;randn(1,DataLen)*10^-6];
for i=1:(DataLen-3)
    vn=[0 0 0];
%     z=[0;0];
    z=Z(:,i);
    if(mod(i,2)==1)
%         att=fscanf(fid1,'%f%f%f\n',[3,1]);
        T=a2Tbn(att((i+1)/2,:));
        F=[0,2*W_ien(3,1),0,-g0,0,T(1,1),T(1,2),0,0,0;
        -2*W_ien(3,1),0,g0,0,0,T(2,1),T(2,2),0,0,0;
        0,-1/Rm,0,W_ien(3,1),-W_ien(2,1),0,0,T(1,1),T(1,2),T(1,3);
        1/Rn,0,-W_ien(3,1),0,0,0,0,T(2,1),T(2,2),T(2,3);
        W_ien(3,1)/W_ien(2,1)/Rn,0,W_ien(2,1),0,0,0,0,T(3,1),T(3,2),T(3,3);
        zeros(5,10)];
    B=eye(10);
    [F,B]=c2d(F,B,t);
    end
    x_half=F*x;
    P_half=F*P*F'+B*Q*B'/t;
    K=P_half*H'*inv(H*P_half*H'+R);
    x=x_half+K*(z-H*x_half);
    P=(eye(10)-K*H)*P_half;
    x_watch(:,i) = x;
end
Q=[H;H*F;H*F*F;H*F*F*F;H*F*F*F*F];
[U,S,V]=svd(Q);
disp('奇异值分解');
disp(S);

phi_e =x(3);
phi_n =x(4);
phi_u =x(5);
Terr = [1 -phi_u phi_n;phi_u 1 -phi_e; -phi_n phi_e 1];


%correct
% k=0;
% for i=1:DataLen
%     vn=[0 0 0];
%     vn_correct(:,i)=[vn(1)-x(1);vn(2)-x(2);vn(3)];
%     if(mod(i,2)==1)
%         k=k+1;
% %         att=fscanf(fid1,'%f%f%f\n',[3,1]);
%         T=a2Tbn(att(k,:));
%         T=Terr*T;
%         att_correct(k,:)=Tbn2Attitude(T);
% %         att_correct(:,k)=[r2d(att_correct(3,k)),r2d(att_correct(1,k)),r2d(att_correct(2,k))];
%     end    
% end

disp('卡尔曼滤波增益阵');
disp(K);
resdisp('Initial align attitudes (arcdeg)',[phi_e*180/pi,phi_n*180/pi,phi_u*180/pi]);