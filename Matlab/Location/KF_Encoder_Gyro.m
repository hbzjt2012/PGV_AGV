clc;
clear;
%第一列数据为编码器角速度
%第二列数据为陀螺仪角速度
%第三列为陀螺仪角度
%第四列为PGV传感器角度
%第五列为时间
data=load('ReceivedTofile-COM4-2018-5-10-20-10-11.txt');

theta_rate_by_encoder=data(:,1)';
theta_rate_by_gyro=data(:,2)';
theta_by_gyro=data(:,3)';
theta_by_pgv=data(:,4)'-data(1,4)';
time=data(:,5)';
theta_rate_target=abs(600*127/4/(534+534)*sin(time / 3.14 * pi));

measure=[theta_by_gyro;theta_rate_by_gyro];%第一列为角度，第二列为角速度
x_KF=measure;
A=[1,0;0,0];
B=[0.02;1];
P=[0,0;0,1];    %初始的协方差矩阵
R=[0.2,0;0,0.08];   %运动噪声
Q=[0.1,0;0,0.02];  %测量噪声


theta_by_cal_use_encoder=zeros(1,length(time));
theta_rate_by_cal_pgv=zeros(1,length(time));

for i=2:length(time)
    theta_by_cal_use_encoder(i)= mod(theta_rate_by_encoder(i)*0.02+ theta_by_cal_use_encoder(i-1),360);
    delta_theta=theta_by_pgv(i)-theta_by_pgv(i-1);
    if delta_theta<-180
        delta_theta=delta_theta+360;
    elseif delta_theta>180
        delta_theta=delta_theta-360;
    end
    theta_rate_by_cal_pgv(i)=delta_theta/0.02;
    x_KF(:,i)=A*x_KF(:,i-1)+B*theta_rate_by_encoder(i);%计算预测值
    P=A*P*A'+R;%计算协方差矩阵预测值
    K=P/(P+Q);  %计算增益
    x_KF(:,i)=x_KF(:,i)+K*(measure(:,i)-x_KF(:,i));%更新预测值
    P=(eye(2)-K)*P; %更新协方差矩阵
    
end


% subplot(1, 2, 1);
% plot(time,theta_by_pgv,'b');
% hold on
% plot(time,theta_by_gyro,'o');
% hold on
% plot(time,theta_by_cal_use_encoder,'g')
% hold on
% plot(time,x_KF(1,:),'k');
% xlim([18 30])
% 
% subplot(1, 2, 2);
plot(time,theta_rate_by_gyro,'b');
hold on
plot(time,theta_rate_by_encoder,'r');
hold on
plot(time,theta_rate_target,'g');
hold on
plot(time,x_KF(2,:),'k');
hold on
% plot(time,theta_rate_by_cal_pgv,'c');
xlim([91 95])

min(theta_by_pgv-x_KF(1,:))
