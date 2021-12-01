function dJ_d = jacobian_dot_D(in1,in2)
%JACOBIAN_DOT_D
%    DJ_D = JACOBIAN_DOT_D(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    30-Nov-2021 01:30:18

dth1 = in1(6,:);
dth2 = in1(7,:);
l_AC = in2(20,:);
l_OB = in2(19,:);
th1 = in1(1,:);
th2 = in1(2,:);
t2 = th1+th2;
t3 = sin(t2);
t4 = cos(t2);
t5 = dth2.*l_AC.*t4;
dJ_d = reshape([-dth1.*(l_AC.*t3+l_OB.*sin(th1))-dth2.*l_AC.*t3,t5+dth1.*(l_AC.*t4+l_OB.*cos(th1)),-dth1.*l_AC.*t3-dth2.*l_AC.*t3,t5+dth1.*l_AC.*t4,0.0,0.0,0.0,0.0,0.0,0.0],[2,5]);