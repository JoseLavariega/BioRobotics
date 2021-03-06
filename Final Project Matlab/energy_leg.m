function E = energy_leg(in1,in2)
%ENERGY_LEG
%    E = ENERGY_LEG(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    30-Nov-2021 01:30:13

I1 = in2(6,:);
I2 = in2(7,:);
I3 = in2(8,:);
I4 = in2(9,:);
I5 = in2(10,:);
Ir = in2(11,:);
N = in2(12,:);
dth1 = in1(6,:);
dth2 = in1(7,:);
dth3 = in1(8,:);
dx = in1(9,:);
dy = in1(10,:);
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
y = in1(5,:);
t2 = sin(th1);
t3 = th1+th2;
t4 = sin(t3);
t5 = l_AC.*t4;
t6 = th1+th2+th3;
t7 = sin(t6);
t8 = l_E_m5.*t7;
t32 = l_OB.*t2;
t9 = dy+dth2.*(t5+t8)+dth1.*(t5+t8+t32+l_DE.*t2)+dth3.*l_E_m5.*t7;
t10 = cos(t3);
t11 = l_AC.*t10;
t12 = cos(th1);
t13 = cos(t6);
t14 = l_E_m5.*t13;
t30 = l_OB.*t12;
t35 = l_DE.*t12;
t15 = dx+dth2.*(t11+t14)+dth1.*(t11+t14+t30+t35)+dth3.*l_E_m5.*t13;
t28 = l_OA.*t2;
t16 = dy+dth1.*(t5+t28+l_C_m4.*t2)+dth2.*l_AC.*t4;
t26 = l_OA.*t12;
t36 = l_C_m4.*t12;
t17 = dx+dth1.*(t11+t26+t36)+dth2.*l_AC.*t10;
t18 = dth1+dth2;
t19 = t18.^2;
t20 = dth1+dth2+N.*dth3;
t21 = dth1.^2;
t22 = dx+dth1.*l_O_m1.*t12;
t23 = dy+dth1.*l_O_m1.*t2;
t24 = dth1+dth2+dth3;
t25 = th3-th3_0;
t37 = l_A_m3.*t10;
t27 = dx+dth1.*(t26+t37)+dth2.*l_A_m3.*t10;
t29 = dy+dth1.*(t28+l_A_m3.*t4)+dth2.*l_A_m3.*t4;
t38 = l_B_m2.*t10;
t31 = dx+dth1.*(t30+t38)+dth2.*l_B_m2.*t10;
t33 = dy+dth1.*(t32+l_B_m2.*t4)+dth2.*l_B_m2.*t4;
t34 = dth1+N.*dth2;
E = (I2.*t19)./2.0+(I1.*t21)./2.0+(I3.*t19)./2.0+(I4.*t21)./2.0+(I5.*t24.^2)./2.0+(Ir.*t20.^2)./2.0+(Ir.*t34.^2)./2.0+(kappa.*t25.^2)./2.0+(m5.*(t9.^2+t15.^2))./2.0+(m4.*(t16.^2+t17.^2))./2.0+(m1.*(t22.^2+t23.^2))./2.0+(m3.*(t27.^2+t29.^2))./2.0+(m2.*(t31.^2+t33.^2))./2.0-g.*m5.*(t11+t14+t30+t35-y)+g.*m1.*(y-l_O_m1.*t12)-g.*m3.*(t26+t37-y)-g.*m2.*(t30+t38-y)-g.*m4.*(t11+t26+t36-y)+(Ir.*N.^2.*t21)./2.0;
