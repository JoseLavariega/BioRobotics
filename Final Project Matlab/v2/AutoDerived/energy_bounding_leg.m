function E = energy_bounding_leg(in1,in2)
%ENERGY_BOUNDING_LEG
%    E = ENERGY_BOUNDING_LEG(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    18-Nov-2021 12:07:00

I1 = in2(6,:);
I2 = in2(7,:);
I3 = in2(8,:);
I4 = in2(9,:);
I5 = in2(10,:);
dth1 = in1(6,:);
dth2 = in1(7,:);
dth3 = in1(8,:);
g = in2(23,:);
kappa = in2(24,:);
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
th3_0 = in2(25,:);
t2 = th1+th2;
t3 = sin(t2);
t7 = sin(th1);
t26 = l_OA.*t7;
t4 = dth1.*(t26+l_A_m3.*t3)+dth2.*l_A_m3.*t3;
t5 = cos(t2);
t9 = cos(th1);
t24 = l_OA.*t9;
t36 = l_A_m3.*t5;
t37 = t24+t36;
t6 = dth1.*t37+dth2.*l_A_m3.*t5;
t19 = l_OB.*t7;
t8 = dth1.*(t19+l_B_m2.*t3)+dth2.*l_B_m2.*t3;
t14 = l_OB.*t9;
t38 = l_B_m2.*t5;
t39 = t14+t38;
t10 = dth1.*t39+dth2.*l_B_m2.*t5;
t11 = dth1+dth2;
t12 = t11.^2;
t13 = l_AC.*t5;
t15 = th1+th2+th3;
t16 = cos(t15);
t17 = l_E_m5.*t16;
t32 = l_DE.*t9;
t33 = t13+t14+t17+t32;
t18 = dth2.*(t13+t17)+dth1.*t33+dth3.*l_E_m5.*t16;
t20 = l_AC.*t3;
t21 = sin(t15);
t22 = l_E_m5.*t21;
t23 = dth2.*(t20+t22)+dth1.*(t19+t20+t22+l_DE.*t7)+dth3.*l_E_m5.*t21;
t34 = l_C_m4.*t9;
t35 = t13+t24+t34;
t25 = dth1.*t35+dth2.*l_AC.*t5;
t27 = dth1.*(t20+t26+l_C_m4.*t7)+dth2.*l_AC.*t3;
t28 = dth1.^2;
t29 = l_O_m1.^2;
t30 = dth1+dth2+dth3;
t31 = th3-th3_0;
E = (I2.*t12)./2.0+(I3.*t12)./2.0+(I1.*t28)./2.0+(I4.*t28)./2.0+(m1.*(t7.^2.*t28.*t29+t9.^2.*t28.*t29))./2.0+(I5.*t30.^2)./2.0+(kappa.*t31.^2)./2.0+(m3.*(t4.^2+t6.^2))./2.0+(m2.*(t8.^2+t10.^2))./2.0+(m5.*(t18.^2+t23.^2))./2.0+(m4.*(t25.^2+t27.^2))./2.0-g.*m5.*t33-g.*m4.*t35-g.*m3.*t37-g.*m2.*t39-g.*l_O_m1.*m1.*t9;
