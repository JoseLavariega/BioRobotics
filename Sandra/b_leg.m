function b = b_leg(in1,in2,in3)
%B_LEG
%    B = B_LEG(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    08-Oct-2021 00:56:45

dth1 = in1(3,:);
dth2 = in1(4,:);
g = in3(19,:);
l_AC = in3(17,:);
l_A_m3 = in3(13,:);
l_B_m2 = in3(12,:);
l_C_m4 = in3(14,:);
l_OA = in3(15,:);
l_OB = in3(16,:);
l_O_m1 = in3(11,:);
m1 = in3(1,:);
m2 = in3(2,:);
m3 = in3(3,:);
m4 = in3(4,:);
tau1 = in2(1,:);
tau2 = in2(2,:);
th1 = in1(1,:);
th2 = in1(2,:);
t2 = cos(th1);
t3 = sin(th1);
t4 = th1+th2;
t5 = l_O_m1.^2;
t6 = l_C_m4.*t2;
t7 = l_OA.*t2;
t8 = l_OB.*t2;
t9 = cos(t4);
t10 = l_C_m4.*t3;
t11 = l_OA.*t3;
t12 = l_OB.*t3;
t13 = sin(t4);
t14 = l_AC.*t9;
t15 = l_A_m3.*t9;
t16 = l_B_m2.*t9;
t17 = l_AC.*t13;
t18 = l_A_m3.*t13;
t19 = l_B_m2.*t13;
t20 = dth1.*t14;
t21 = dth2.*t14;
t22 = dth1.*t15;
t23 = dth2.*t15;
t24 = dth1.*t16;
t25 = dth2.*t16;
t26 = dth1.*t17;
t27 = dth2.*t17;
t28 = dth1.*t18;
t29 = dth2.*t18;
t30 = dth1.*t19;
t31 = dth2.*t19;
t32 = t7+t15;
t33 = t8+t16;
t34 = t11+t18;
t35 = t12+t19;
t43 = t6+t7+t14;
t47 = t10+t11+t17;
t36 = dth1.*t32;
t37 = dth1.*t33;
t38 = dth1.*t34;
t39 = dth1.*t35;
t40 = t20+t21;
t41 = t22+t23;
t42 = t24+t25;
t44 = t26+t27;
t45 = t28+t29;
t46 = t30+t31;
t48 = dth1.*t47;
t49 = dth1.*t43;
t50 = t23+t36;
t51 = t25+t37;
t52 = t29+t38;
t53 = t31+t39;
t54 = t21+t49;
t55 = t27+t48;
t56 = t18.*t50.*4.0;
t57 = t19.*t51.*4.0;
t58 = t15.*t52.*2.0;
t59 = t16.*t53.*2.0;
t60 = t17.*t54.*4.0;
t61 = t14.*t55.*2.0;
mt1 = [tau1.*2.0+dth2.*((m3.*(t56-t58-t34.*t41.*2.0+t32.*t45.*4.0))./2.0+(m2.*(t57-t59-t35.*t42.*2.0+t33.*t46.*4.0))./2.0+(m4.*(t60-t61-t40.*t47.*2.0+t43.*t44.*4.0))./2.0)+dth1.*((m3.*(t32.*t52.*2.0+t34.*t50.*2.0))./2.0+(m2.*(t33.*t53.*2.0+t35.*t51.*2.0))./2.0+(m4.*(t43.*t55.*2.0+t47.*t54.*2.0))./2.0+dth1.*m1.*t2.*t3.*t5.*2.0)-g.*m2.*t35-g.*m3.*t34-g.*m4.*t47-m3.*t50.*t52-m2.*t51.*t53-m4.*t54.*t55-g.*l_O_m1.*m1.*t3-dth1.^2.*m1.*t2.*t3.*t5];
mt2 = [tau2.*2.0+dth2.*((m3.*(t56-t58-t18.*t41.*2.0+t15.*t45.*4.0))./2.0+(m4.*(t60-t61-t17.*t40.*2.0+t14.*t44.*4.0))./2.0+(m2.*(t57-t59-t19.*t42.*2.0+t16.*t46.*4.0))./2.0)+(m3.*(t41.*t52.*2.0-t45.*t50.*4.0))./2.0+(m2.*(t42.*t53.*2.0-t46.*t51.*4.0))./2.0+(m4.*(t40.*t55.*2.0-t44.*t54.*4.0))./2.0+dth1.*((m3.*(t58+t18.*t50.*2.0))./2.0+(m2.*(t59+t19.*t51.*2.0))./2.0+(m4.*(t61+t17.*t54.*2.0))./2.0)-g.*m2.*t19-g.*m3.*t18-g.*m4.*t17];
b = reshape([mt1,mt2],2,1);