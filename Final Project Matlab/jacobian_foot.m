function J_f = jacobian_foot(in1,in2)
%JACOBIAN_FOOT
%    J_F = JACOBIAN_FOOT(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    30-Nov-2021 01:30:18

l_AC = in2(20,:);
l_DE = in2(21,:);
l_EF = in2(22,:);
l_OB = in2(19,:);
th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
t2 = cos(th1);
t3 = th1+th2;
t4 = cos(t3);
t5 = l_AC.*t4;
t6 = th1+th2+th3;
t7 = cos(t6);
t8 = l_EF.*t7;
t9 = sin(th1);
t10 = sin(t3);
t11 = l_AC.*t10;
t12 = sin(t6);
t13 = l_EF.*t12;
J_f = reshape([t5+t8+l_DE.*t2+l_OB.*t2,t11+t13+l_DE.*t9+l_OB.*t9,t5+t8,t11+t13,t8,t13,1.0,0.0,0.0,1.0],[2,5]);
