function J_b = jacobian_B(in1,in2)
%JACOBIAN_B
%    J_B = JACOBIAN_B(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    29-Nov-2021 13:56:42

l_OB = in2(19,:);
th1 = in1(1,:);
J_b = reshape([l_OB.*cos(th1),l_OB.*sin(th1),0.0,0.0,0.0,0.0,1.0,0.0,0.0,1.0],[2,5]);
