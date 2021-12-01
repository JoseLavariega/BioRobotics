function b = b_bounding_leg(in1,in2,in3)
%B_BOUNDING_LEG
%    B = B_BOUNDING_LEG(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    18-Nov-2021 12:00:56

dth1 = in1(6,:);
dth2 = in1(7,:);
dth3 = in1(8,:);
g = in3(23,:);
kappa = in3(24,:);
l_AC = in3(20,:);
l_A_m3 = in3(15,:);
l_B_m2 = in3(14,:);
l_C_m4 = in3(16,:);
l_DE = in3(21,:);
l_E_m5 = in3(17,:);
l_OA = in3(18,:);
l_OB = in3(19,:);
l_O_m1 = in3(13,:);
m1 = in3(1,:);
m2 = in3(2,:);
m3 = in3(3,:);
m4 = in3(4,:);
m5 = in3(5,:);
tau1 = in2(1,:);
tau2 = in2(2,:);
tau3 = in2(3,:);
th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
th3_0 = in3(25,:);
t2 = th1+th2;
t3 = sin(t2);
t4 = cos(th1);
t5 = cos(t2);
t6 = sin(th1);
t7 = l_AC.*t5;
t8 = l_C_m4.*t4;
t9 = l_OA.*t4;
t10 = t7+t8+t9;
t11 = dth2.*l_AC.*t5;
t12 = l_AC.*t3;
t13 = l_C_m4.*t6;
t14 = l_OA.*t6;
t15 = t12+t13+t14;
t16 = dth2.*l_AC.*t3;
t17 = l_A_m3.*t5;
t18 = t9+t17;
t19 = dth2.*l_A_m3.*t5;
t20 = l_A_m3.*t3;
t21 = t14+t20;
t22 = dth2.*l_A_m3.*t3;
t23 = l_B_m2.*t5;
t24 = l_OB.*t4;
t25 = t23+t24;
t26 = dth2.*l_B_m2.*t5;
t27 = l_B_m2.*t3;
t28 = l_OB.*t6;
t29 = t27+t28;
t30 = dth2.*l_B_m2.*t3;
t31 = th1+th2+th3;
t32 = cos(t31);
t33 = l_E_m5.*t32;
t34 = t7+t33;
t35 = dth2.*t34;
t36 = dth3.*l_E_m5.*t32;
t37 = sin(t31);
t38 = l_E_m5.*t37;
t39 = t12+t38;
t40 = l_DE.*t4;
t41 = t7+t24+t33+t40;
t42 = l_DE.*t6;
t43 = t12+t28+t38+t42;
t44 = dth2.*t39;
t45 = dth3.*l_E_m5.*t37;
t46 = dth1.*t41;
t47 = t35+t36+t46;
t48 = dth1.*t43;
t49 = t44+t45+t48;
t50 = dth1.*t39;
t51 = t44+t45+t50;
t52 = dth1.*t34;
t53 = t35+t36+t52;
t54 = dth1.*l_A_m3.*t3;
t55 = t22+t54;
t56 = dth1.*t18;
t57 = t19+t56;
t58 = dth1.*l_A_m3.*t5;
t59 = t19+t58;
t60 = dth1.*t21;
t61 = t22+t60;
t62 = dth1.*l_B_m2.*t3;
t63 = t30+t62;
t64 = dth1.*t25;
t65 = t26+t64;
t66 = dth1.*l_B_m2.*t5;
t67 = t26+t66;
t68 = dth1.*t29;
t69 = t30+t68;
t70 = t34.*t49.*2.0;
t71 = l_A_m3.*t3.*t57.*2.0;
t72 = l_B_m2.*t3.*t65.*2.0;
t73 = dth1.*l_AC.*t5;
t74 = t11+t73;
t75 = dth1.*t10;
t76 = t11+t75;
t77 = dth1.*l_AC.*t3;
t78 = t16+t77;
t79 = dth1.*t15;
t80 = t16+t79;
t81 = l_AC.*t3.*t76.*2.0;
t82 = dth1.*l_E_m5.*t32;
t83 = dth2.*l_E_m5.*t32;
t84 = t36+t82+t83;
t85 = dth1.*l_E_m5.*t37;
t86 = dth2.*l_E_m5.*t37;
t87 = t45+t85+t86;
t88 = l_E_m5.*t32.*t49.*2.0;
b = [tau1+tau2+tau3+dth2.*(m5.*(t70-t39.*t47.*2.0-t41.*t51.*2.0+t43.*t53.*2.0).*(-1.0./2.0)+(m4.*(t81+t10.*t78.*2.0-t15.*t74.*2.0-l_AC.*t5.*t80.*2.0))./2.0+(m3.*(t71+t18.*t55.*2.0-t21.*t59.*2.0-l_A_m3.*t5.*t61.*2.0))./2.0+(m2.*(t72+t25.*t63.*2.0-t29.*t67.*2.0-l_B_m2.*t5.*t69.*2.0))./2.0)-(dth3.*m5.*(t88+t43.*t84.*2.0-t41.*t87.*2.0-l_E_m5.*t37.*t47.*2.0))./2.0-g.*m4.*t15-g.*m3.*t21-g.*m2.*t29-g.*m5.*t43-g.*l_O_m1.*m1.*t6;tau2+tau3-(m5.*(t47.*t51.*2.0-t49.*t53.*2.0))./2.0-(m3.*(t55.*t57.*2.0-t59.*t61.*2.0))./2.0-(m2.*(t63.*t65.*2.0-t67.*t69.*2.0))./2.0+(m4.*(t74.*t80.*2.0-t76.*t78.*2.0))./2.0+dth2.*(m5.*(t70-t34.*t51.*2.0-t39.*t47.*2.0+t39.*t53.*2.0).*(-1.0./2.0)+(m4.*(t81-l_AC.*t3.*t74.*2.0+l_AC.*t5.*t78.*2.0-l_AC.*t5.*t80.*2.0))./2.0+(m3.*(t71+l_A_m3.*t5.*t55.*2.0-l_A_m3.*t3.*t59.*2.0-l_A_m3.*t5.*t61.*2.0))./2.0+(m2.*(t72+l_B_m2.*t5.*t63.*2.0-l_B_m2.*t3.*t67.*2.0-l_B_m2.*t5.*t69.*2.0))./2.0)-(dth3.*m5.*(t88-t34.*t87.*2.0+t39.*t84.*2.0-l_E_m5.*t37.*t47.*2.0))./2.0-g.*m5.*t39-g.*l_AC.*m4.*t3-g.*l_A_m3.*m3.*t3-g.*l_B_m2.*m2.*t3;(m5.*(t49.*t84.*2.0-t47.*t87.*2.0))./2.0-(kappa.*(th3.*2.0-th3_0.*2.0))./2.0-(dth2.*m5.*(t88-l_E_m5.*t32.*t51.*2.0-l_E_m5.*t37.*t47.*2.0+l_E_m5.*t37.*t53.*2.0))./2.0-(dth3.*m5.*(t88-l_E_m5.*t37.*t47.*2.0-l_E_m5.*t32.*t87.*2.0+l_E_m5.*t37.*t84.*2.0))./2.0-g.*l_E_m5.*m5.*t37;0.0;0.0];
