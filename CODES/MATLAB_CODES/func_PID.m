function der_state=func_PID(~,state,Kp1,Ki1,Kd1,Kp2,Ki2,Kd2,q1_des,q2_des,m1,m2,l1,l2,g)

q1=state(1);
q2=state(2);
q1_dot=state(3);
q2_dot=state(4);
int_e1=state(5);
int_e2=state(6);

M11=(m1+m2)*l1^2+m2*l2*(l2+2*l1*cos(q2));
M12=m2*l2*(l2+l1*cos(q2));
M22=m2*l2^2;
M=[M11,M12;M12,M22];

C11=-m2*l1*l2*sin(q2)*q2_dot;
C12=-m2*l1*l2*sin(q2)*(q1_dot+q2_dot);
C21=0;
C22=m2*l1*l2*sin(q2)*q1_dot;
C=[C11,C12;C21,C22];

G1=m1*l1*g*cos(q1)+m2*g*(l2*cos(q1+q2)+l1*cos(q1));
G2=m2*g*l2*cos(q1+q2);
G=[G1;G2];

e1=q1_des-q1;
e2=q2_des-q2;

tau1=Kp1*e1+Ki1*int_e1-Kd1*q1_dot;
tau2=Kp2*e2+Ki2*int_e2-Kd2*q2_dot;
Tau=[tau1;tau2];

q_dot_dot=M\(Tau-C*[q1_dot;q2_dot]-G);

der_state=[q1_dot;q2_dot;q_dot_dot(1);q_dot_dot(2);e1;e2];
end