function J_c = jacobian_C(in1,in2)
%JACOBIAN_C
%    J_C = JACOBIAN_C(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    30-Nov-2021 01:30:17

l_AC = in2(20,:);
l_OA = in2(18,:);
th1 = in1(1,:);
th2 = in1(2,:);
t2 = th1+th2;
t3 = cos(t2);
t4 = l_AC.*t3;
t5 = sin(t2);
t6 = l_AC.*t5;
J_c = reshape([t4+l_OA.*cos(th1),t6+l_OA.*sin(th1),t4,t6,0.0,0.0,1.0,0.0,0.0,1.0],[2,5]);
