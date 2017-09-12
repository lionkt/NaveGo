clear all;
clc;
echo off;
%===============================
%����ģ��
%===============================
% �������
simTime=100;      %�����������
T=1;                     %����ʱ��
w2=3*2*pi/360;     %ģ��2ת����3��
w3=-3*2*pi/360;    %ģ��3ת����-3��
H=[1,0,0,0;0,0,1,0];                      %ģ���������
G=[T^2/2,0;T,0;0,T^2/2;0,T];              %ģ�͹���������Ȩ����
r=200;                                 %20 2000
R=[r,0;0,r];                            %ģ����������Э�������
Q=[10,0;0,10];                                  %ģ�͹�������Э�������

F1=[1,T,0,0;0,1,0,0;0,0,1,T;0,0,0,1];     %ģ��1״̬ת�ƾ���

F2=[1,sin(w2*T)/w2,0,(cos(w2*T)-1)/w2;
    0,cos(w2*T),0,sin(w2*T);
    0,(1-cos(w2*T))/w2,1,sin(w2*T)/w2;
    0,-sin(w2*T),0,cos(w2*T)];            %ģ��2״̬ת�ƾ��� ��ת��

F3=[1,sin(w3*T)/w3,0,(cos(w3*T)-1)/w3;
    0,cos(w3*T),0,sin(w3*T);
    0,(1-cos(w3*T))/w3,1,sin(w3*T)/w3;
    0,-sin(w3*T),0,cos(w3*T)];            %ģ��3״̬ת�ƾ��� ��ת��

w4=3*2*pi/360;     %ģ��2ת����3��
w5=-3*2*pi/360;    %ģ��3ת����-3��

F4=[1,sin(w4*T)/w4,0,(cos(w4*T)-1)/w4;
    0,cos(w4*T),0,sin(w4*T);
    0,(1-cos(w4*T))/w4,1,sin(w4*T)/w4;
    0,-sin(w4*T),0,cos(w4*T)];            %ģ��2״̬ת�ƾ���

F5=[1,sin(w5*T)/w5,0,(cos(w5*T)-1)/w5;
    0,cos(w5*T),0,sin(w5*T);
    0,(1-cos(w5*T))/w5,1,sin(w5*T)/w5;
    0,-sin(w5*T),0,cos(w5*T)];            %ģ��3״̬ת�ƾ���

x0=[1000,200,1000,200]';  % ��ʼ״̬

% ������������
%[z,z_true]=targets();
randn('state',sum(100*clock)); % Shuffle the pack!
x = zeros(4,simTime);
z = zeros(2,simTime);         %��������������
z_true = zeros(2,simTime);    %��ֵ����
measureNoise = zeros(2,simTime);
measureNoise = 3*sqrt(R)*randn(2,simTime);  %������������
x(:,1)=x0;
z(:,1)=H*x(:,1)+measureNoise(:,1);
z_true(:,1)=H*x(:,1);
for a=2:simTime
    if (a>=20)&&(a<=40) 
        x(:,a)=F4*x(:,a-1);      %20--40s��ת 
    elseif (a>=60)&&(a<=80) 
       x(:,a)=F5*x(:,a-1);        %60--80s��ת 
    else
        x(:,a)=F1*x(:,a-1);      %����ֱ���˶�
    end; 
z(:,a)=H*x(:,a)+measureNoise(:,a);
z_true(:,a)=H*x(:,a);
end;
%===============================
%     IMM
%===============================

%��ʼ��
x1_IMM = zeros(4,1);      %ģ��1IMM�㷨״̬����ֵ
x2_IMM = zeros(4,1);      %ģ��2IMM�㷨״̬����ֵ
x3_IMM = zeros(4,1);      %ģ��3IMM�㷨״̬����ֵ
x_pro_IMM = zeros(4,simTime);   %IMM�㷨ģ���ۺ�״̬����ֵ
P_IMM=zeros(4,4,simTime);       %IMM�㷨ģ���ۺ�״̬Э�������
P1_IMM=zeros(4,4);
P2_IMM=zeros(4,4);
P3_IMM=zeros(4,4);              %IMM�㷨��ģ��Э�������
r1_IMM=zeros(2,1);
r2_IMM=zeros(2,1);
r3_IMM=zeros(2,1);
S1_IMM=zeros(2,2);
S2_IMM=zeros(2,2);
S3_IMM=zeros(2,2);


%��ʼ��
x_pro_IMM(:,1)=x0;

pij=[0.9,0.05,0.05;
    0.1,0.8,0.1;
    0.05,0.15,0.8];    %ģ��ת�Ƹ��ʾ���
%pij=[0.6,0.2,0.2;0.2,0.6,0.2;0.25,0.15,0.6];    %ģ��ת�Ƹ��ʾ���
u_IMM=zeros(3,simTime);
u_IMM(:,1)=[0.3,0.3,0.4]';  %IMM�㷨ģ�͸���

x1_IMM=x0;x2_IMM=x0;x3_IMM=x0;  %IMM�㷨��ģ�ͳ�ʼ״̬

P0=diag([1000,500,1000,500]);  %��ʼ״̬Э�������

P1_IMM=P0;P2_IMM=P0;P3_IMM=P0;

P_IMM(:,:,1)=P0;

%main loop

for t=1:simTime-1
    
    %��һ��Interacting��ֻ���IMM�㷨��
    c_j=pij'*u_IMM(:,t);
    
    ui1=(1/c_j(1))*pij(:,1).*u_IMM(:,t);
    ui2=(1/c_j(2))*pij(:,2).*u_IMM(:,t);
    ui3=(1/c_j(3))*pij(:,3).*u_IMM(:,t);    %����ģ�ͻ�ϸ���
    
    % �����ģ���˲���ʼ������
    x01=x1_IMM*ui1(1)+x2_IMM*ui1(2)+x3_IMM*ui1(3);
    x02=x1_IMM*ui2(1)+x2_IMM*ui2(2)+x3_IMM*ui2(3);
    x03=x1_IMM*ui3(1)+x2_IMM*ui3(2)+x3_IMM*ui3(3);   %��ģ���˲���ʼ״̬
    
    P01=(P1_IMM+[x1_IMM-x01]*[x1_IMM-x01]')*ui1(1)+...
        (P2_IMM+[x2_IMM-x01]*[x2_IMM-x01]')*ui1(2)+...
        (P3_IMM+[x3_IMM-x01]*[x3_IMM-x01]')*ui1(3);
    P02=(P1_IMM+[x1_IMM-x02]*[x1_IMM-x02]')*ui2(1)+...
        (P2_IMM+[x2_IMM-x02]*[x2_IMM-x02]')*ui2(2)+...
        (P3_IMM+[x3_IMM-x02]*[x3_IMM-x02]')*ui2(3);
    P03=(P1_IMM+[x1_IMM-x03]*[x1_IMM-x03]')*ui3(1)+...
        (P2_IMM+[x2_IMM-x03]*[x2_IMM-x03]')*ui3(2)+...
        (P3_IMM+[x3_IMM-x03]*[x3_IMM-x03]')*ui3(3);  %��ģ���˲���ʼ״̬Э�������
    
    %�ڶ���--�������˲�
    %ģ��1�������˲�
    [x1_IMM,P1_IMM,r1_IMM,S1_IMM]=Kalman(x01,P01,z(:,t+1),F1,G,Q,H,R);
    %ģ��2�������˲�
    [x2_IMM,P2_IMM,r2_IMM,S2_IMM]=Kalman(x02,P02,z(:,t+1),F2,G,Q,H,R);
    %ģ��3�������˲�
    [x3_IMM,P3_IMM,r3_IMM,S3_IMM]=Kalman(x03,P03,z(:,t+1),F3,G,Q,H,R);
    
    %������--ģ�͸��ʸ���
    [u_IMM(:,t+1)]=Model_P_up(r1_IMM,r2_IMM,r3_IMM,S1_IMM,S2_IMM,S3_IMM,c_j);
    
    %���Ĳ�--ģ���ۺ�
    [x_pro_IMM(:,t+1),P_IMM(:,:,t+1)]=Model_mix(x1_IMM,x2_IMM,x3_IMM,P1_IMM,P2_IMM,P3_IMM,u_IMM(:,t));
end
%===============================
%��ͼ
%===============================
%Ŀ��켣
figure(1)
plot(z_true(1,:),z_true(2,:)); grid on; hold on
plot(x_pro_IMM(1,:),x_pro_IMM(3,:),'r');
plot(z(1,:),z(2,:),'*'); 
hold off
title('Ŀ���˶��켣');
xlabel('x/m'); ylabel('y/m');
legend('��ʵֵ','�˲�ֵ','����ֵ');
text(z(1,1)+500,z(2,1),'t=1');

% λ�����
figure(2)
subplot(2,1,1);
t=1:simTime;
plot(t,abs(x_pro_IMM(1,t)-x(1,t)),'LineWidth',2);grid on
title('x����λ�ø������');
xlabel('t/s'); ylabel('x-error/m');

subplot(2,1,2);
t=1:simTime;
plot(t,abs(x_pro_IMM(3,t)-x(3,t)),'LineWidth',2);grid on
title('y����λ�ø������');
xlabel('t/s'); ylabel('y-error/m');

% �ٶ����
figure(3)
subplot(2,1,1);
t=1:simTime;
plot(t,abs(x_pro_IMM(2,t)-x(2,t)),'LineWidth',2);grid on
title('x�����ٶȸ������');
xlabel('t/s'); ylabel('vx-error/m');

subplot(2,1,2);
t=1:simTime;
plot(t,abs(x_pro_IMM(4,t)-x(4,t)),'LineWidth',2);grid on
title('y�����ٶȸ������');
xlabel('t/s'); ylabel('vy-error/m');

% ģ�͸���
figure(4)
plot(t,u_IMM(1,t),'k:',t,u_IMM(2,t),'r-.',t,u_IMM(3,t),'b--','LineWidth',2);grid on
title('IMM�㷨ģ�͸�������');
xlabel('t/s'); ylabel('ģ�͸���');
legend('ģ��1','ģ��2','ģ��3');