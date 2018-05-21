syms x y theta velocity velocity_angle time

A=[x
    y
    theta]
B=[velocity*time*cos(velocity_angle+theta)
    velocity*time*sin(velocity_angle+theta)
    0];

C=A+B

G=diff(C(1),velocity)