function [coor,variance] = Update_Coor_Line(coor,velocity,time_s,variance )

% syms x y theta velocity velocity_angle time
% 
% g(x,y,theta,velocity,velocity_angle,time)=[x+velocity*cos(velocity_angle)*time
%     y+velocity*sin(velocity_angle)*time
%     theta]
% 
A=coor;
angle_temp=(coor(3)+velocity(2))/180*pi();

%¸üÐÂÔ¤²âÖµ
A(1)=A(1)+velocity(1)*cos(angle_temp)*time_s;
A(2)=A(2)+velocity(1)*sin(angle_temp)*time_s;

G=[1 0 -time_s*velocity*sin(angle_temp)
    0 1 time_s*velocity*cos(angle_temp)
    0 0 1];

R=[2 0 0
    0 1 0
    0 0 1];

V=[time*cos(theta + velocity_angle) 
end

