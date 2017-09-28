clc;
clear all;
close all;
%data=xlsread('CANOE9.csv');
load params;
%data = load('odo_1505809803273.txt');
%data = load('odo_1505809925344.txt');
%data = load('2017-09-20-17-19-58_odo.txt');
%data = load('2017-09-20-17-21-15_odo.txt');
%ins = load('2017-09-20-17-21-15_ins.txt');

%data = load('2017-09-20-17-28-09_odo.txt');
%ins = load('2017-09-20-17-28-09_ins.txt');

data = load('2017-09-20-17-48-25_odo.txt');
ins = load('2017-09-20-17-48-25_ins.txt');
tS=data(:,1);
PS=data(:,2);
LS=data(:,5);
RS=data(:,6);

HPR = findPitchRoll(tS,ins,0);  % �õ�heading��pithch��row

index = find(LS == 0);
NLS = LS;
for i=2:size(index,1)-1
    NLS(index(i)) = (LS( index(i) - 1) + LS( index(i) + 1))/2;
end


for i=2:size(NLS,1)-1
    if( ( ( (NLS(i+1) - NLS(i)) > 0.1 ) && ((NLS(i-1) - NLS(i)) > 0.1) ) )
        
        NLS(i) = (NLS( i - 1) + NLS( i + 1))/2;
    end
end

for i=2:size(NLS,1)-1
    if( ( ( (NLS(i+1) - NLS(i)) < 0.1 ) && ((NLS(i-1) - NLS(i)) < 0.1) ) )
        
        NLS(i) = (NLS( i - 1) + NLS( i + 1))/2;
    end
end

LS = NLS;


index = find(RS == 0);
NRS = RS;
for i=2:size(index,1)-1
    NRS(index(i)) = (RS( index(i) - 1) + RS( index(i) + 1))/2;
end

for i=2:size(NRS,1)-1
    if( ( ( (NRS(i+1) - NRS(i)) > 0.1 ) && ((NRS(i-1) - NRS(i)) > 0.1) ) )
        
        NRS(i) = (NRS( i - 1) + NRS( i + 1))/2;
    end
end

for i=2:size(NRS,1)-1
    if( ( ( (NRS(i+1) - NRS(i)) < 0.1 ) && ((NRS(i-1) - NRS(i)) < 0.1) ) )
        
        NRS(i) = (NRS( i - 1) + NRS( i + 1))/2;
    end
end

RS = NRS;


LS = LS.*PS;
RS = RS.*PS;


%1.033
speedscale = params(1);
scale = params(2);

t=tS/1000;
L=LS*0.27777778*scale*speedscale;
R=RS*0.27777778*speedscale; %100125



tiredis=1.575;



start=[-0.676,0,0,0,0,pi/2];
startN=[-1.352,0,0,0,0,pi/2];
len=length(t);
position=zeros(len+1,2);
altitude=zeros(len+1,2);
positionN=zeros(len+1,2);
altitudeN=zeros(len+1,2);
altitude_3d = zeros(len+1,3);   % crown add for 3d
position_3d = zeros(len+1,3);   % crown add for 3d
seta_s=zeros(len,1);
seta_sN=zeros(len,1);
altitude(1,1)=(HPR(1) + 90) *pi/180;
altitude_3d(1,1)=(HPR(1) + 90) *pi/180; % crown add for 3d
altitude_3d(1,2:3) = deg2rad(HPR(1,2:3));   % crown add for 3d
position(1,1)=0;
position_3d(1,1) = 0;   % crown add for 3d

%%%% 2d��̬����
% for i=1:1:len-2
%     T=t(i+1)-t(i);
%     if abs(L(i)-R(i))<0.0000000001
%         altitude(i+1,1)=altitude(i,1);
%         altitude(i+1,2)=altitude(i,2);
%         position(i+1,1)=position(i,1)+(L(i)+R(i))/2*T*cos(altitude(i));
%         position(i+1,2)=position(i,2)+(L(i)+R(i))/2*T*sin(altitude(i));
%     elseif (L(i)-R(i))>0.0000000001
% 
%         %seta=(L(i)*T-R(i)*T)/tiredis;
%         seta = (HPR(i+1,1) - HPR(i,1))*3.1415926/180;
%         altitude(i+1,1)=altitude(i,1)+seta;
% 
%         r=R(i)*T/(seta+1e-6);
%         x=position(i,1);
%         y=position(i,2);
%         x0=x+(r+tiredis/2)*sin(altitude(i,1));
%         y0=y-(r+tiredis/2)*cos(altitude(i,1));
%         position(i+1,1)=(x-x0)*cos(seta)+(y-y0)*sin(seta)+x0;
%         position(i+1,2)=(y-y0)*cos(seta)-(x-x0)*sin(seta)+y0;
%     else
%         %seta=(L(i)*T-R(i)*T)/tiredis;
%         seta = (HPR(i+1,1) - HPR(i,1))*3.1415926/180;
%         altitude(i+1,1)=altitude(i,1)+seta;
% 
%         r=L(i)*T/(seta+1e-6);
%         x=position(i,1);
%         y=position(i,2);
%         x0=x+(r+tiredis/2)*sin(altitude(i,1));
%         y0=y-(r+tiredis/2)*cos(altitude(i,1));
%         position(i+1,1)=(x-x0)*cos(seta)+(y-y0)*sin(seta)+x0;
%         position(i+1,2)=(y-y0)*cos(seta)-(x-x0)*sin(seta)+y0;
%     end
% end

%%%% 3d��̬����
%%% ǿԼ����Vb = Vb_y
%%% ǿԼ�������֡����ֵķ������һ����ֻ��ת�ٲ�һ��
mode_selection_threshold = 0.0000000001;   % ������������ٲ�С��mode_selection_threshold����Ϊֱ�С��ʼ��0.0000000001
for i=1:1:len-2
    T=t(i+1)-t(i);
%     Cnb = angle2dcm(-deg2rad(HPR(i,3)),-deg2rad(HPR(i,2)),-deg2rad(HPR(i,1)+90),'YXZ');  %����roll��pitch��heading��˳������y��x��z��˳��ת
    Cnb = angle2dcm(deg2rad(HPR(i,3)),deg2rad(HPR(i,2)),deg2rad(HPR(i,1)+90),'YXZ');
    Cbn = Cnb^-1;
%     Cnb_n = angle2dcm(-deg2rad(HPR(i+1,3)),-deg2rad(HPR(i+1,2)),-deg2rad(HPR(i+1,1)+90),'YXZ');
    Cnb_n = angle2dcm(deg2rad(HPR(i+1,3)),deg2rad(HPR(i+1,2)),deg2rad(HPR(i+1,1)+90),'YXZ');
    Cbn_n = Cnb_n^-1;
    v1_dir = Cbn(:,2);   %��ʱ�̵ĳ������ٶȷ���
    v2_dir = Cbn_n(:,2); %��һʱ�̵ĳ������ٶȷ���
    if abs(L(i)-R(i))<mode_selection_threshold   %���ֱ��
        %        altitude_3d(i+1,:) = altitude_3d(i,:);
        v_norm = (L(i)+R(i))/2; %�ٶȵ�ģ
        position_3d(i+1,:) = (v_norm*T *v1_dir/norm(v1_dir))' + position_3d(i,:);  %��ԭ���ķ���ǰ��
    else    % �����������ת��
        rot_axes_in_N = cross(v1_dir, v2_dir);   %����ת�����ת��
        rot_axes_in_N = rot_axes_in_N/norm(rot_axes_in_N);  %��λ��
        theta = acos(dot(v1_dir,v2_dir)/(norm(v1_dir)*norm(v2_dir)));  % arccos��ȡֵ��Χ��0~pi
        if (L(i)-R(i))>=mode_selection_threshold
            %%% �����ת��
            r = R(i)*T/(theta+1e-6);  % ���ֵ�ת��뾶
            r_real = r + tiredis/2;   % ������ת��뾶
            a_dir = cross(rot_axes_in_N, v1_dir);    % ��p1ָ����ת���ĵľ�������
            a_dir = a_dir/norm(a_dir);  % ��λ��
            p1_N = position_3d(i,:);   % �����ϸ�ʱ�̵�λ��
            p0_N = p1_N + r_real*(a_dir(:))';    % ��ת���ĵ�����
            % ��p0Ϊ���ģ�p1-p0Ϊx�ᣬp0+rot_nΪZ�������ϵ�µ�P2��ת��Nϵ��
            rot_X_axes_in_N = (p1_N-p0_N)/norm(p1_N-p0_N);   % ��תϵ��X����Nϵ�µı�ʾ��Ҫ��λ��
            rot_Z_axes_in_N = (rot_axes_in_N(:))';    % ��תϵ��Z����Nϵ�µı�ʾ��Ҫ��λ��
            rot_Z_axes_in_N = rot_Z_axes_in_N/norm(rot_Z_axes_in_N);
            rot_Y_axes_in_N = cross(rot_Z_axes_in_N, rot_X_axes_in_N);   % ��תϵ��Y����Nϵ�µı�ʾ��Ҫ��λ��
            C_rot_N = [rot_X_axes_in_N(:), rot_Y_axes_in_N(:), rot_Z_axes_in_N(:)];
            p2_rot = [r_real*cos(theta), r_real*sin(theta), 0];    % p2��rotϵ�µı�ʾ
            p2_N = (C_rot_N*p2_rot(:))' + p0_N;     % ��rotϵ�µ�p2ת��Nϵ�±�ʾ
            position_3d(i+1,:) = p2_N(:)';
            %            altitude_3d(i+1,:) =
        else
            %%% �����ת��
            r = L(i)*T/(theta+1e-6);  % ���ֵ�ת��뾶
            r_real = r + tiredis/2;   % ������ת��뾶
            a_dir = cross(rot_axes_in_N, v1_dir);    % ��p1ָ����ת���ĵľ�������
            a_dir = a_dir/norm(a_dir);  % ��λ��
            p1_N = position_3d(i,:);   % �����ϸ�ʱ�̵�λ��
            p0_N = p1_N + r_real*(a_dir(:))';    % ��ת���ĵ�����
            % ��p0Ϊ���ģ�p1-p0Ϊx�ᣬp0+rot_nΪZ�������ϵ�µ�P2��ת��Nϵ��
            rot_X_axes_in_N = (p1_N-p0_N)/norm(p1_N-p0_N);   % ��תϵ��X����Nϵ�µı�ʾ��Ҫ��λ��
            rot_Z_axes_in_N = (rot_axes_in_N(:))';    % ��תϵ��Z����Nϵ�µı�ʾ��Ҫ��λ��
            rot_Z_axes_in_N = rot_Z_axes_in_N/norm(rot_Z_axes_in_N);
            rot_Y_axes_in_N = cross(rot_Z_axes_in_N, rot_X_axes_in_N);   % ��תϵ��Y����Nϵ�µı�ʾ��Ҫ��λ��
            rot_Y_axes_in_N = rot_Y_axes_in_N/norm(rot_Y_axes_in_N);
            C_rot_N = [rot_X_axes_in_N(:), rot_Y_axes_in_N(:), rot_Z_axes_in_N(:)];
            p2_rot = [r_real*cos(theta), r_real*sin(theta), 0];    % p2��rotϵ�µı�ʾ
            p2_N = (C_rot_N*p2_rot(:))' + p0_N;     % ��rotϵ�µ�p2ת��Nϵ�±�ʾ
            position_3d(i+1,:) = p2_N(:)';
            %            altitude_3d(i+1,:) =
        end  
    end
end



position(end,:)=[];
positionN(end,:)=[];
position_x0=position(:,1) * -1;
position_y0=position(:,2) * 1;

% crown add for 3d
position_3d(end,:)=[];

xuanzhuan=0*3.1415925/180;
position_x=position_x0*cos(xuanzhuan)+position_y0*sin(xuanzhuan);
position_y=position_y0*cos(xuanzhuan)-position_x0*sin(xuanzhuan);

distance = sqrt((position_x(1) - position_x(end-1))^2 + (position_y(1) - position_y(end-1))^2)

%% plot
% figure;
% plot(position_x,position_y,'r.' , 'LineWidth',8);
% axis equal;

figure;
plot3(position_3d(:,1),position_3d(:,2),position_3d(:,3));
hold on;
plot3(position_3d(1,1),position_3d(1,2),position_3d(1,3),'o','linewidth',2);
axis equal;
grid on;

figure;
plot(tS,LS);
hold on;
plot(tS,RS);
hold off;


