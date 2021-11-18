function rE = position_foot(in1,in2)
%POSITION_FOOT
%    RE = POSITION_FOOT(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    18-Nov-2021 12:07:01

l_AC = in2(20,:);
l_DE = in2(21,:);
l_OB = in2(19,:);
th1 = in1(1,:);
th2 = in1(2,:);
t2 = sin(th1);
t3 = th1+th2;
t4 = cos(th1);
rE = [l_DE.*t2+l_OB.*t2+l_AC.*sin(t3);-l_DE.*t4-l_OB.*t4-l_AC.*cos(t3)];
