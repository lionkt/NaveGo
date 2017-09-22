function [X,P,e,S]=Kalman(X_Forward,P_Forward,Z,A,G,Q,H,R)
%�������˲�2012.2.27   IMMר�ã��������в�ͬ
%����˵��
%       Z--�۲�����ʸ��
%       A--ϵͳģ��״̬����
%       G--ϵͳģ������ϵ������
%       Q--ϵͳģ����������
%       H--����ϵ������
%       R--����ģ������Э����
%       X_Forward--ǰ�ι���״̬ʸ��
%       P_Forward--ǰ�ι���״̬Э�������

%       X--�������״̬ʸ��
%       P--�������״̬Э�������
%       e--�в�
%       S--�в�Э�������

% Ԥ��
X_Pre=A*X_Forward;
P_Pre=A*P_Forward*A'+G*Q*G';

% �������
K=P_Pre*H'*inv(H*P_Pre*H'+R)';

% Pzz = H*P_Forward*H'+ R;                   %S(k+1/k+1) ��ϢЭ����
% Pxz = P_Forward*H' ;                       %״̬������֮���Э����
% K = P_Forward*H'*(inv(Pzz));               %K(k+1) ����
    
e = Z-H*(A*X_Forward);
S=H*P_Pre*H'+R;  %�в�Э�������

% �����˲�ֵ�����Э������
X=A*X_Forward+K*(Z-H*(A*X_Forward));

M=K*H;
n=size(M);
I=eye(n);
P=(I-K*H)*P_Pre*(I-K*H)'+ K*R*K';
end