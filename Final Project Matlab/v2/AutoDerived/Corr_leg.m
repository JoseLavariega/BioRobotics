function Corr_Joint_Sp = Corr_leg(in1,in2)
%CORR_LEG
%    CORR_JOINT_SP = CORR_LEG(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    19-Nov-2021 00:33:52

dth1 = in1(6,:);
dth2 = in1(7,:);
dth3 = in1(8,:);
l_AC = in2(20,:);
l_A_m3 = in2(15,:);
l_B_m2 = in2(14,:);
l_C_m4 = in2(16,:);
l_DE = in2(21,:);
l_E_m5 = in2(17,:);
l_OA = in2(18,:);
l_OB = in2(19,:);
m2 = in2(2,:);
m3 = in2(3,:);
m4 = in2(4,:);
m5 = in2(5,:);
th2 = in1(2,:);
th3 = in1(3,:);
t2 = th2+th3;
t3 = sin(t2);
t4 = dth2.^2;
t5 = dth3.^2;
t6 = sin(th2);
t7 = sin(th3);
t8 = dth1.^2;
Corr_Joint_Sp = [-l_AC.*l_C_m4.*m4.*t4.*t6-l_AC.*l_DE.*m5.*t4.*t6-l_AC.*l_E_m5.*m5.*t5.*t7-l_DE.*l_E_m5.*m5.*t3.*t4-l_DE.*l_E_m5.*m5.*t3.*t5-l_AC.*l_OA.*m4.*t4.*t6-l_AC.*l_OB.*m5.*t4.*t6-l_A_m3.*l_OA.*m3.*t4.*t6-l_B_m2.*l_OB.*m2.*t4.*t6-l_E_m5.*l_OB.*m5.*t3.*t4-l_E_m5.*l_OB.*m5.*t3.*t5-dth1.*dth2.*l_AC.*l_C_m4.*m4.*t6.*2.0-dth1.*dth2.*l_AC.*l_DE.*m5.*t6.*2.0-dth1.*dth3.*l_AC.*l_E_m5.*m5.*t7.*2.0-dth2.*dth3.*l_AC.*l_E_m5.*m5.*t7.*2.0-dth1.*dth2.*l_DE.*l_E_m5.*m5.*t3.*2.0-dth1.*dth3.*l_DE.*l_E_m5.*m5.*t3.*2.0-dth2.*dth3.*l_DE.*l_E_m5.*m5.*t3.*2.0-dth1.*dth2.*l_AC.*l_OA.*m4.*t6.*2.0-dth1.*dth2.*l_AC.*l_OB.*m5.*t6.*2.0-dth1.*dth2.*l_A_m3.*l_OA.*m3.*t6.*2.0-dth1.*dth2.*l_B_m2.*l_OB.*m2.*t6.*2.0-dth1.*dth2.*l_E_m5.*l_OB.*m5.*t3.*2.0-dth1.*dth3.*l_E_m5.*l_OB.*m5.*t3.*2.0-dth2.*dth3.*l_E_m5.*l_OB.*m5.*t3.*2.0;l_AC.*l_C_m4.*m4.*t6.*t8+l_AC.*l_DE.*m5.*t6.*t8-l_AC.*l_E_m5.*m5.*t5.*t7+l_DE.*l_E_m5.*m5.*t3.*t8+l_AC.*l_OA.*m4.*t6.*t8+l_AC.*l_OB.*m5.*t6.*t8+l_A_m3.*l_OA.*m3.*t6.*t8+l_B_m2.*l_OB.*m2.*t6.*t8+l_E_m5.*l_OB.*m5.*t3.*t8-dth1.*dth3.*l_AC.*l_E_m5.*m5.*t7.*2.0-dth2.*dth3.*l_AC.*l_E_m5.*m5.*t7.*2.0;l_E_m5.*m5.*(l_AC.*t4.*t7+l_AC.*t7.*t8+l_DE.*t3.*t8+l_OB.*t3.*t8+dth1.*dth2.*l_AC.*t7.*2.0);0.0;0.0];
