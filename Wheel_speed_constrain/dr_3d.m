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

HPR = findPitchRoll(tS,ins,0);

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
seta_s=zeros(len,1);
seta_sN=zeros(len,1);
altitude(1,1)=(HPR(1) + 90) *pi/180;
position(1,1)=0;


for i=1:1:len-2
    T=t(i+1)-t(i);
    if abs(L(i)-R(i))<0.0000000001
        altitude(i+1,1)=altitude(i,1);
        altitude(i+1,2)=altitude(i,2);
        position(i+1,1)=position(i,1)+(L(i)+R(i))/2*T*cos(altitude(i));
        position(i+1,2)=position(i,2)+(L(i)+R(i))/2*T*sin(altitude(i));
    elseif (L(i)-R(i))>0.0000000001
        
        %seta=(L(i)*T-R(i)*T)/tiredis;
        seta = (HPR(i+1,1) - HPR(i,1))*3.1415926/180;
        altitude(i+1,1)=altitude(i,1)+seta;
        
        r=R(i)*T/(seta+1e-6);
        x=position(i,1);
        y=position(i,2);
        x0=x+(r+tiredis/2)*sin(altitude(i,1));
        y0=y-(r+tiredis/2)*cos(altitude(i,1));
        position(i+1,1)=(x-x0)*cos(seta)+(y-y0)*sin(seta)+x0;
        position(i+1,2)=(y-y0)*cos(seta)-(x-x0)*sin(seta)+y0;
    else
        %seta=(L(i)*T-R(i)*T)/tiredis;
        seta = (HPR(i+1,1) - HPR(i,1))*3.1415926/180;
        altitude(i+1,1)=altitude(i,1)+seta;
        
        r=L(i)*T/(seta+1e-6);
        x=position(i,1);
        y=position(i,2);
        x0=x+(r+tiredis/2)*sin(altitude(i,1));
        y0=y-(r+tiredis/2)*cos(altitude(i,1));
        position(i+1,1)=(x-x0)*cos(seta)+(y-y0)*sin(seta)+x0;
        position(i+1,2)=(y-y0)*cos(seta)-(x-x0)*sin(seta)+y0;
    end
end
position(end,:)=[];
positionN(end,:)=[];
position_x0=position(:,1) * -1;
position_y0=position(:,2) * 1;

xuanzhuan=0*3.1415925/180;
position_x=position_x0*cos(xuanzhuan)+position_y0*sin(xuanzhuan);
position_y=position_y0*cos(xuanzhuan)-position_x0*sin(xuanzhuan);

distance = sqrt((position_x(1) - position_x(end-1))^2 + (position_y(1) - position_y(end-1))^2)

plot(position_x,position_y,'r.' , 'LineWidth',8);
axis equal;

figure;
plot(tS,LS);
hold on;
plot(tS,RS);
hold off;


