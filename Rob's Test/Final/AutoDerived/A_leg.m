function A = A_leg(in1,in2)
%A_LEG
%    A = A_LEG(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    02-Dec-2021 00:21:51

I1 = in2(6,:);
I2 = in2(7,:);
I3 = in2(8,:);
I4 = in2(9,:);
I5 = in2(10,:);
Ir = in2(11,:);
N = in2(12,:);
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
t35 = Ir.*N;
t36 = l_OB.*t2.*2.0;
t37 = l_AC.*t8.*2.0;
t38 = l_OA.*t2.*2.0;
t39 = l_AC.*t11.*2.0;
t40 = l_OA.*t4.*2.0;
t41 = l_OB.*t4.*2.0;
t42 = l_A_m3.*t11.*t13.*2.0;
t43 = l_A_m3.*t8.*t10.*2.0;
t44 = t42+t43;
t45 = (m3.*t44)./2.0;
t46 = l_B_m2.*t11.*t15.*2.0;
t47 = l_B_m2.*t8.*t14.*2.0;
t48 = t46+t47;
t49 = (m2.*t48)./2.0;
t50 = l_AC.*t5.*t8.*2.0;
t51 = l_AC.*t7.*t11.*2.0;
t52 = t50+t51;
t53 = (m4.*t52)./2.0;
t54 = t16+t30;
t55 = t18.*t54.*2.0;
t56 = t19+t33;
t57 = t22.*t56.*2.0;
t58 = t55+t57;
t59 = (m5.*t58)./2.0;
t60 = I2+I3+I5+Ir+t35+t45+t49+t53+t59;
t61 = N.^2;
t62 = Ir.*t61;
t63 = l_AC.^2;
t64 = t8.^2;
t65 = l_A_m3.^2;
t66 = t11.^2;
t67 = l_B_m2.^2;
t68 = l_E_m5.*t29.*2.0;
t69 = l_E_m5.*t32.*2.0;
t70 = l_E_m5.*t22.*t32.*2.0;
t71 = l_E_m5.*t18.*t29.*2.0;
t72 = t70+t71;
t73 = (m5.*t72)./2.0;
t74 = I5+t35+t73;
t75 = l_E_m5.*t29.*t54.*2.0;
t76 = l_E_m5.*t32.*t56.*2.0;
t77 = t75+t76;
t78 = (m5.*t77)./2.0;
t79 = I5+t35+t78;
t80 = l_E_m5.^2;
t81 = l_A_m3.*t8.*2.0;
t82 = t38+t81;
t83 = (m3.*t82)./2.0;
t84 = l_B_m2.*t8.*2.0;
t85 = t36+t84;
t86 = (m2.*t85)./2.0;
t87 = l_DE.*t2.*2.0;
t88 = t36+t37+t68+t87;
t89 = (m5.*t88)./2.0;
t90 = l_C_m4.*t2.*2.0;
t91 = t37+t38+t90;
t92 = (m4.*t91)./2.0;
t93 = l_O_m1.*m1.*t2;
t94 = t83+t86+t89+t92+t93;
t95 = t37+t68;
t96 = (m5.*t95)./2.0;
t97 = l_AC.*m4.*t8;
t98 = l_A_m3.*m3.*t8;
t99 = l_B_m2.*m2.*t8;
t100 = t96+t97+t98+t99;
t101 = l_E_m5.*m5.*t29;
t102 = l_DE.*t4.*2.0;
t103 = t39+t41+t69+t102;
t104 = (m5.*t103)./2.0;
t105 = l_C_m4.*t4.*2.0;
t106 = t39+t40+t105;
t107 = (m4.*t106)./2.0;
t108 = l_A_m3.*t11.*2.0;
t109 = t40+t108;
t110 = (m3.*t109)./2.0;
t111 = l_B_m2.*t11.*2.0;
t112 = t41+t111;
t113 = (m2.*t112)./2.0;
t114 = l_O_m1.*m1.*t4;
t115 = t104+t107+t110+t113+t114;
t116 = t39+t69;
t117 = (m5.*t116)./2.0;
t118 = l_AC.*m4.*t11;
t119 = l_A_m3.*m3.*t11;
t120 = l_B_m2.*m2.*t11;
t121 = t117+t118+t119+t120;
t122 = l_E_m5.*m5.*t32;
t123 = m1+m2+m3+m4+m5;
A = reshape([I1+I2+I3+I4+I5+Ir.*2.0+t62+(m1.*(t2.^2.*t3.*2.0+t3.*t4.^2.*2.0))./2.0+(m4.*(t5.^2.*2.0+t7.^2.*2.0))./2.0+(m3.*(t10.^2.*2.0+t13.^2.*2.0))./2.0+(m2.*(t14.^2.*2.0+t15.^2.*2.0))./2.0+(m5.*(t18.^2.*2.0+t22.^2.*2.0))./2.0,t60,t74,t94,t115,t60,I2+I3+I5+Ir+t62+(m5.*(t54.^2.*2.0+t56.^2.*2.0))./2.0+(m4.*(t63.*t64.*2.0+t63.*t66.*2.0))./2.0+(m3.*(t64.*t65.*2.0+t65.*t66.*2.0))./2.0+(m2.*(t64.*t67.*2.0+t66.*t67.*2.0))./2.0,t79,t100,t121,t74,t79,I5+t62+(m5.*(t29.^2.*t80.*2.0+t32.^2.*t80.*2.0))./2.0,t101,t122,t94,t100,t101,t123,0.0,t115,t121,t122,0.0,t123],[5,5]);