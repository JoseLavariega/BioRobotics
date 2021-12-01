function dJ_e = jacobian_dot_E(in1,in2)
%JACOBIAN_DOT_E
%    DJ_E = JACOBIAN_DOT_E(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    29-Nov-2021 13:56:43

dth1 = in1(6,:);
dth2 = in1(7,:);
l_AC = in2(20,:);
l_DE = in2(21,:);
l_OB = in2(19,:);
th1 = in1(1,:);
th2 = in1(2,:);
t2 = cos(th1);
t3 = sin(th1);
t4 = th1+th2;
t5 = cos(t4);
t6 = sin(t4);
t7 = dth2.*l_AC.*t5;
t8 = dth2.*l_AC.*t6;
t9 = -t8;
dJ_e = reshape([t9-dth1.*(l_AC.*t6+l_DE.*t3+l_OB.*t3),t7+dth1.*(l_AC.*t5+l_DE.*t2+l_OB.*t2),t9-dth1.*l_AC.*t6,t7+dth1.*l_AC.*t5,0.0,0.0,0.0,0.0,0.0,0.0],[2,5]);
