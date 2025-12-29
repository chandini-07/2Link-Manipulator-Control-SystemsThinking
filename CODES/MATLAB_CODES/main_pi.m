clc;
clear;
close all;

q1_des=0;
q2_des=0;
q1_0=0.2;
q2_0=0.15;
q1_dot_0=0;
q2_dot_0=0;
int_e1_0=0;
int_e2_0=0;

m1=5;
m2=3;
l1=0.25;
l2=0.15;
g=9.81;

Kp1=1500;
Ki1=120;
Kp2=1000;
Ki2=150;

y0=[q1_0;q2_0;q1_dot_0;q2_dot_0;int_e1_0;int_e2_0];

t_span=0:0.01:10;

[t,y]=ode45(@(t,s)func_PI(t,s,Kp1,Ki1,Kp2,Ki2,q1_des,q2_des,m1,m2,l1,l2,g),t_span,y0);

q1=y(:,1);
q2=y(:,2);

figure;
subplot(2,1,1);
plot(t,q1,'r','LineWidth',1.8);
xlabel('Time (s)');
ylabel('q1 (rad)');
title(['Joint 1 Angle vs Time (Kp1=',num2str(Kp1),', Ki1=',num2str(Ki1),')']);
grid on;

subplot(2,1,2);
plot(t,q2,'b','LineWidth',1.8);
xlabel('Time (s)');
ylabel('q2 (rad)');
title(['Joint 2 Angle vs Time (Kp2=',num2str(Kp2),', Ki2=',num2str(Ki2),')']);
grid on;

sgtitle('2-Link Manipulator using PI Control');

error_q1=q1_des-q1;
error_q2=q2_des-q2;

figure;
subplot(2,1,1);
plot(t,error_q1,'r','LineWidth',1.8);
xlabel('Time (s)');
ylabel('Error in q1 (rad)');
title(['Tracking Error for Joint 1 (Kp1=',num2str(Kp1),', Ki1=',num2str(Ki1),')']);
grid on;

subplot(2,1,2);
plot(t,error_q2,'b','LineWidth',1.8);
xlabel('Time (s)');
ylabel('Error in q2 (rad)');
title(['Tracking Error for Joint 2 (Kp2=',num2str(Kp2),', Ki2=',num2str(Ki2),')']);
grid on;

sgtitle('PI Controller Tracking Errors');