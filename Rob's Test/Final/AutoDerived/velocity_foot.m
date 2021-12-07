function drE = velocity_foot(in1,in2)
%VELOCITY_FOOT
%    DRE = VELOCITY_FOOT(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    02-Dec-2021 00:21:57

dth1 = in1(6,:);
dth2 = in1(7,:);
dx = in1(9,:);
dy = in1(10,:);
l_AC = in2(20,:);
l_DE = in2(21,:);
l_OB = in2(19,:);
th1 = in1(1,:);
th2 = in1(2,:);
t2 = cos(th1);
t3 = th1+th2;
t4 = cos(t3);
t5 = sin(th1);
t6 = sin(t3);
drE = [dx+dth1.*(l_AC.*t4+l_DE.*t2+l_OB.*t2)+dth2.*l_AC.*t4;dy+dth1.*(l_AC.*t6+l_DE.*t5+l_OB.*t5)+dth2.*l_AC.*t6];