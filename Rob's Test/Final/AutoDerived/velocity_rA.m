function drA = velocity_rA(in1,in2)
%VELOCITY_RA
%    DRA = VELOCITY_RA(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    02-Dec-2021 00:21:56

dth1 = in1(6,:);
dx = in1(9,:);
dy = in1(10,:);
l_OA = in2(18,:);
th1 = in1(1,:);
drA = [dx+dth1.*l_OA.*cos(th1);dy+dth1.*l_OA.*sin(th1)];
