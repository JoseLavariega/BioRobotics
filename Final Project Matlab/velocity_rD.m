function drD = velocity_rD(in1,in2)
%VELOCITY_RD
%    DRD = VELOCITY_RD(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    30-Nov-2021 01:30:15

dth1 = in1(6,:);
dth2 = in1(7,:);
dx = in1(9,:);
dy = in1(10,:);
l_AC = in2(20,:);
l_OB = in2(19,:);
th1 = in1(1,:);
th2 = in1(2,:);
t2 = th1+th2;
t3 = cos(t2);
t4 = sin(t2);
drD = [dx+dth1.*(l_AC.*t3+l_OB.*cos(th1))+dth2.*l_AC.*t3;dy+dth1.*(l_AC.*t4+l_OB.*sin(th1))+dth2.*l_AC.*t4];
