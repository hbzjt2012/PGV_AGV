clc;
clear;
step_length=0.01;
radius=10;
velocity_angle=pi()/5;
length=1000;

%系数
c1=2;
c2=1;
c3=1;
time=linspace(0,step_length*length,length);
target_coor=zeros(6,length);%目标坐标、速度
current_coor=zeros(6,length);%当前坐标、速度
error_coor=zeros(6,length);%坐标、速度误差

current_coor(1,1)=2;
current_coor(2,1)=2;
current_coor(3,1)=1;

 %迭代
for i=1:length
   
    if i~=1
    current_coor(1,i)= current_coor(1,i-1)+current_coor(4,i-1)*step_length;
    current_coor(2,i)= current_coor(2,i-1)+current_coor(5,i-1)*step_length;
    current_coor(3,i)= current_coor(3,i-1)+current_coor(6,i-1)*step_length;        
    end
    target_coor(3,i)=velocity_angle*time(1,i);%  角速度1
    target_coor(1,i)=radius*cos(target_coor(3,i));
    target_coor(2,i)=radius*sin(target_coor(3,i));
    target_coor(6,i)=velocity_angle;
    target_coor(4,i)=-radius*sin(target_coor(3,i))*velocity_angle;
    target_coor(5,i)=radius*cos(target_coor(3,i))*velocity_angle;
      
    error_coor(1,i)=target_coor(1,i)-current_coor(1,i);
    error_coor(2,i)=target_coor(2,i)-current_coor(2,i);
    error_coor(3,i)=target_coor(3,i)-current_coor(3,i);
    
    current_coor(4,i)=target_coor(4,i)+c1*error_coor(1,i);
    current_coor(5,i)=target_coor(5,i)+c2*error_coor(2,i);
    current_coor(6,i)=target_coor(6,i)+c3*(error_coor(3,i));
    if current_coor(4,i)>8
        current_coor(4,i)=8;
    elseif current_coor(4,i)<-8
        current_coor(4,i)=-8;
    end
    
    if current_coor(5,i)>8
        current_coor(5,i)=8;
    elseif current_coor(5,i)<-8
        current_coor(5,i)=-8;
    end
      
%     current_coor(6,i)=target_coor(6,i)+c3*sin(error_coor(3,i));
    
end

subplot(3,1,1);
plot(current_coor(1,:),current_coor(2,:),'r')
hold on
plot(target_coor(1,:),target_coor(2,:))
axis([-15,15,-15,15]);

subplot(3,1,2);
plot(time,error_coor(1,:),'b');
hold on
plot(time,error_coor(2,:),'r');
hold on
plot(time,error_coor(3,:),'-- g');
axis([0,10,-3,10]);

subplot(3,1,3);
plot(time,current_coor(4,:),'b');
hold on
plot(time,target_coor(4,:),'r');
% plot(time,current_coor(5,:),'r');
% hold on
% plot(time,current_coor(6,:),'-- g');
% axis([0,10,-3,10]);

