function A = A_bounding_leg(in1,in2)
%A_BOUNDING_LEG
%    A = A_BOUNDING_LEG(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    18-Nov-2021 12:06:58

I1 = in2(6,:);
I2 = in2(7,:);
I3 = in2(8,:);
I4 = in2(9,:);
I5 = in2(10,:);
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
t3 = l_O_m1.^2;
t4 = sin(th1);
t6 = th1+th2;
t8 = cos(t6);
t9 = l_OA.*t2;
t16 = l_AC.*t8;
t27 = l_C_m4.*t2;
t5 = t9+t16+t27;
t11 = sin(t6);
t12 = l_OA.*t4;
t19 = l_AC.*t11;
t28 = l_C_m4.*t4;
t7 = t12+t19+t28;
t24 = l_A_m3.*t8;
t10 = t9+t24;
t23 = l_A_m3.*t11;
t13 = t12+t23;
t17 = l_OB.*t2;
t26 = l_B_m2.*t8;
t14 = t17+t26;
t20 = l_OB.*t4;
t25 = l_B_m2.*t11;
t15 = t20+t25;
t21 = th1+th2+th3;
t29 = cos(t21);
t30 = l_E_m5.*t29;
t31 = l_DE.*t2;
t18 = t16+t17+t30+t31;
t32 = sin(t21);
t33 = l_E_m5.*t32;
t34 = l_DE.*t4;
t22 = t19+t20+t33+t34;
t35 = l_A_m3.*t11.*t13.*2.0;
t36 = l_A_m3.*t8.*t10.*2.0;
t37 = t35+t36;
t38 = (m3.*t37)./2.0;
t39 = l_B_m2.*t11.*t15.*2.0;
t40 = l_B_m2.*t8.*t14.*2.0;
t41 = t39+t40;
t42 = (m2.*t41)./2.0;
t43 = l_AC.*t5.*t8.*2.0;
t44 = l_AC.*t7.*t11.*2.0;
t45 = t43+t44;
t46 = (m4.*t45)./2.0;
t47 = t16+t30;
t48 = t18.*t47.*2.0;
t49 = t19+t33;
t50 = t22.*t49.*2.0;
t51 = t48+t50;
t52 = (m5.*t51)./2.0;
t53 = I2+I3+I5+t38+t42+t46+t52;
t54 = l_AC.^2;
t55 = t8.^2;
t56 = l_A_m3.^2;
t57 = t11.^2;
t58 = l_B_m2.^2;
t59 = l_E_m5.*t22.*t32.*2.0;
t60 = l_E_m5.*t18.*t29.*2.0;
t61 = t59+t60;
t62 = (m5.*t61)./2.0;
t63 = I5+t62;
t64 = l_E_m5.*t29.*t47.*2.0;
t65 = l_E_m5.*t32.*t49.*2.0;
t66 = t64+t65;
t67 = (m5.*t66)./2.0;
t68 = I5+t67;
t69 = l_E_m5.^2;
A = reshape([I1+I2+I3+I4+I5+(m1.*(t2.^2.*t3.*2.0+t3.*t4.^2.*2.0))./2.0+(m4.*(t5.^2.*2.0+t7.^2.*2.0))./2.0+(m3.*(t10.^2.*2.0+t13.^2.*2.0))./2.0+(m2.*(t14.^2.*2.0+t15.^2.*2.0))./2.0+(m5.*(t18.^2.*2.0+t22.^2.*2.0))./2.0,t53,t63,0.0,0.0,t53,I2+I3+I5+(m5.*(t47.^2.*2.0+t49.^2.*2.0))./2.0+(m4.*(t54.*t55.*2.0+t54.*t57.*2.0))./2.0+(m3.*(t55.*t56.*2.0+t56.*t57.*2.0))./2.0+(m2.*(t55.*t58.*2.0+t57.*t58.*2.0))./2.0,t68,0.0,0.0,t63,t68,I5+(m5.*(t29.^2.*t69.*2.0+t32.^2.*t69.*2.0))./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[5,5]);
