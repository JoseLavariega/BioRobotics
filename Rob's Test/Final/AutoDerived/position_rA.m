function rA = position_rA(in1,in2)
%POSITION_RA
%    RA = POSITION_RA(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    02-Dec-2021 00:21:56

l_OA = in2(18,:);
th1 = in1(1,:);
x = in1(4,:);
y = in1(5,:);
rA = [x+l_OA.*sin(th1);y-l_OA.*cos(th1)];