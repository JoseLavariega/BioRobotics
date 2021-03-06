function Corr_Joint_Sp = Corr_leg(in1,in2)
%CORR_LEG
%    CORR_JOINT_SP = CORR_LEG(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    29-Nov-2021 13:56:43

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
l_O_m1 = in2(13,:);
m1 = in2(1,:);
m2 = in2(2,:);
m3 = in2(3,:);
m4 = in2(4,:);
m5 = in2(5,:);
th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
t2 = cos(th1);
t3 = sin(th1);
t4 = sin(th2);
t5 = sin(th3);
t6 = th1+th2;
t7 = th2+th3;
t8 = dth1.^2;
t9 = dth2.^2;
t10 = dth3.^2;
t11 = cos(t6);
t12 = sin(t6);
t13 = sin(t7);
t14 = t6+th3;
t17 = l_AC.*l_E_m5.*m5.*t5.*t10;
t18 = dth1.*dth3.*l_AC.*l_E_m5.*m5.*t5.*2.0;
t19 = dth2.*dth3.*l_AC.*l_E_m5.*m5.*t5.*2.0;
t15 = cos(t14);
t16 = sin(t14);
t20 = -t18;
t21 = -t19;
t22 = -t17;
Corr_Joint_Sp = [t20+t21+t22-l_AC.*l_C_m4.*m4.*t4.*t9-l_AC.*l_DE.*m5.*t4.*t9-l_DE.*l_E_m5.*m5.*t9.*t13-l_DE.*l_E_m5.*m5.*t10.*t13-l_AC.*l_OA.*m4.*t4.*t9-l_AC.*l_OB.*m5.*t4.*t9-l_A_m3.*l_OA.*m3.*t4.*t9-l_B_m2.*l_OB.*m2.*t4.*t9-l_E_m5.*l_OB.*m5.*t9.*t13-l_E_m5.*l_OB.*m5.*t10.*t13-dth1.*dth2.*l_AC.*l_C_m4.*m4.*t4.*2.0-dth1.*dth2.*l_AC.*l_DE.*m5.*t4.*2.0-dth1.*dth2.*l_DE.*l_E_m5.*m5.*t13.*2.0-dth1.*dth3.*l_DE.*l_E_m5.*m5.*t13.*2.0-dth2.*dth3.*l_DE.*l_E_m5.*m5.*t13.*2.0-dth1.*dth2.*l_AC.*l_OA.*m4.*t4.*2.0-dth1.*dth2.*l_AC.*l_OB.*m5.*t4.*2.0-dth1.*dth2.*l_A_m3.*l_OA.*m3.*t4.*2.0-dth1.*dth2.*l_B_m2.*l_OB.*m2.*t4.*2.0-dth1.*dth2.*l_E_m5.*l_OB.*m5.*t13.*2.0-dth1.*dth3.*l_E_m5.*l_OB.*m5.*t13.*2.0-dth2.*dth3.*l_E_m5.*l_OB.*m5.*t13.*2.0;t20+t21+t22+l_AC.*l_C_m4.*m4.*t4.*t8+l_AC.*l_DE.*m5.*t4.*t8+l_DE.*l_E_m5.*m5.*t8.*t13+l_AC.*l_OA.*m4.*t4.*t8+l_AC.*l_OB.*m5.*t4.*t8+l_A_m3.*l_OA.*m3.*t4.*t8+l_B_m2.*l_OB.*m2.*t4.*t8+l_E_m5.*l_OB.*m5.*t8.*t13;l_E_m5.*m5.*(l_AC.*t5.*t8+l_AC.*t5.*t9+l_DE.*t8.*t13+l_OB.*t8.*t13+dth1.*dth2.*l_AC.*t5.*2.0);-l_AC.*m4.*t8.*t12-l_AC.*m4.*t9.*t12-l_AC.*m5.*t8.*t12-l_AC.*m5.*t9.*t12-l_A_m3.*m3.*t8.*t12-l_A_m3.*m3.*t9.*t12-l_B_m2.*m2.*t8.*t12-l_B_m2.*m2.*t9.*t12-l_C_m4.*m4.*t3.*t8-l_DE.*m5.*t3.*t8-l_E_m5.*m5.*t8.*t16-l_E_m5.*m5.*t9.*t16-l_E_m5.*m5.*t10.*t16-l_OA.*m3.*t3.*t8-l_OB.*m2.*t3.*t8-l_OA.*m4.*t3.*t8-l_OB.*m5.*t3.*t8-l_O_m1.*m1.*t3.*t8-dth1.*dth2.*l_AC.*m4.*t12.*2.0-dth1.*dth2.*l_AC.*m5.*t12.*2.0-dth1.*dth2.*l_A_m3.*m3.*t12.*2.0-dth1.*dth2.*l_B_m2.*m2.*t12.*2.0-dth1.*dth2.*l_E_m5.*m5.*t16.*2.0-dth1.*dth3.*l_E_m5.*m5.*t16.*2.0-dth2.*dth3.*l_E_m5.*m5.*t16.*2.0;l_AC.*m4.*t8.*t11+l_AC.*m4.*t9.*t11+l_AC.*m5.*t8.*t11+l_AC.*m5.*t9.*t11+l_A_m3.*m3.*t8.*t11+l_A_m3.*m3.*t9.*t11+l_B_m2.*m2.*t8.*t11+l_B_m2.*m2.*t9.*t11+l_C_m4.*m4.*t2.*t8+l_DE.*m5.*t2.*t8+l_E_m5.*m5.*t8.*t15+l_E_m5.*m5.*t9.*t15+l_E_m5.*m5.*t10.*t15+l_OA.*m3.*t2.*t8+l_OB.*m2.*t2.*t8+l_OA.*m4.*t2.*t8+l_OB.*m5.*t2.*t8+l_O_m1.*m1.*t2.*t8+dth1.*dth2.*l_AC.*m4.*t11.*2.0+dth1.*dth2.*l_AC.*m5.*t11.*2.0+dth1.*dth2.*l_A_m3.*m3.*t11.*2.0+dth1.*dth2.*l_B_m2.*m2.*t11.*2.0+dth1.*dth2.*l_E_m5.*m5.*t15.*2.0+dth1.*dth3.*l_E_m5.*m5.*t15.*2.0+dth2.*dth3.*l_E_m5.*m5.*t15.*2.0];
