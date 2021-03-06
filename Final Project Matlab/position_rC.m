function rC = position_rC(in1,in2)
%POSITION_RC
%    RC = POSITION_RC(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    30-Nov-2021 01:30:14

l_AC = in2(20,:);
l_OA = in2(18,:);
th1 = in1(1,:);
th2 = in1(2,:);
x = in1(4,:);
y = in1(5,:);
t2 = th1+th2;
rC = [x+l_AC.*sin(t2)+l_OA.*sin(th1);y-l_AC.*cos(t2)-l_OA.*cos(th1)];
