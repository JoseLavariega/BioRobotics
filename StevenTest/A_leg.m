function A = A_leg(in1,in2)
%A_LEG
%    A = A_LEG(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    29-Nov-2021 13:56:40

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
t3 = sin(th1);
t4 = th1+th2;
t5 = N.^2;
t6 = l_AC.^2;
t7 = l_A_m3.^2;
t8 = l_B_m2.^2;
t9 = l_E_m5.^2;
t10 = l_O_m1.^2;
t11 = Ir.*N;
t41 = m1+m2+m3+m4+m5;
t12 = l_C_m4.*t2;
t13 = l_DE.*t2;
t14 = l_OA.*t2;
t15 = l_OB.*t2;
t16 = cos(t4);
t17 = l_C_m4.*t3;
t18 = l_DE.*t3;
t19 = l_OA.*t3;
t20 = l_OB.*t3;
t21 = sin(t4);
t22 = t4+th3;
t23 = l_O_m1.*m1.*t2;
t25 = l_O_m1.*m1.*t3;
t27 = Ir.*t5;
t24 = cos(t22);
t26 = sin(t22);
t28 = t12.*2.0;
t29 = t13.*2.0;
t30 = t14.*2.0;
t31 = t15.*2.0;
t32 = t17.*2.0;
t33 = t18.*2.0;
t34 = t19.*2.0;
t35 = t20.*2.0;
t36 = t16.^2;
t37 = t21.^2;
t38 = l_AC.*t16;
t39 = l_A_m3.*t16;
t40 = l_B_m2.*t16;
t42 = l_AC.*t21;
t43 = l_A_m3.*t21;
t44 = l_B_m2.*t21;
t45 = t38.*2.0;
t46 = t39.*2.0;
t47 = t40.*2.0;
t48 = t42.*2.0;
t49 = t43.*2.0;
t50 = t44.*2.0;
t51 = m4.*t38;
t52 = m3.*t39;
t53 = m2.*t40;
t54 = l_E_m5.*t24;
t55 = m4.*t42;
t56 = m3.*t43;
t57 = m2.*t44;
t58 = l_E_m5.*t26;
t63 = t14+t39;
t64 = t15+t40;
t65 = t19+t43;
t66 = t20+t44;
t73 = t12+t14+t38;
t74 = t17+t19+t42;
t59 = m5.*t54;
t60 = m5.*t58;
t61 = t54.*2.0;
t62 = t58.*2.0;
t67 = t30+t46;
t68 = t31+t47;
t69 = t34+t49;
t70 = t35+t50;
t71 = t38+t54;
t72 = t42+t58;
t77 = t28+t30+t45;
t78 = t49.*t65;
t79 = t50.*t66;
t82 = t32+t34+t48;
t85 = t46.*t63;
t86 = t47.*t64;
t91 = t45.*t73;
t93 = t48.*t74;
t75 = t45+t61;
t76 = t48+t62;
t80 = (m3.*t67)./2.0;
t81 = (m2.*t68)./2.0;
t83 = (m3.*t69)./2.0;
t84 = (m2.*t70)./2.0;
t89 = t13+t15+t71;
t90 = t18+t20+t72;
t92 = t61.*t71;
t94 = t62.*t72;
t95 = (m4.*t82)./2.0;
t96 = (m4.*t77)./2.0;
t103 = t78+t85;
t104 = t79+t86;
t111 = t91+t93;
t87 = (m5.*t75)./2.0;
t88 = (m5.*t76)./2.0;
t97 = t29+t31+t75;
t98 = t33+t35+t76;
t99 = t62.*t90;
t100 = t61.*t89;
t105 = t71.*t89.*2.0;
t106 = t72.*t90.*2.0;
t107 = (m3.*t103)./2.0;
t108 = (m2.*t104)./2.0;
t112 = t92+t94;
t113 = (m4.*t111)./2.0;
t101 = (m5.*t98)./2.0;
t102 = (m5.*t97)./2.0;
t109 = t51+t52+t53+t87;
t110 = t55+t56+t57+t88;
t114 = (m5.*t112)./2.0;
t116 = t99+t100;
t119 = t105+t106;
t115 = I5+t11+t114;
t117 = (m5.*t116)./2.0;
t120 = (m5.*t119)./2.0;
t121 = t25+t83+t84+t95+t101;
t122 = t23+t80+t81+t96+t102;
t118 = I5+t11+t117;
t123 = I2+I3+I5+Ir+t11+t107+t108+t113+t120;
A = reshape([I1+I2+I3+I4+I5+Ir.*2.0+t27+(m1.*(t2.^2.*t10.*2.0+t3.^2.*t10.*2.0))./2.0+(m3.*(t63.^2.*2.0+t65.^2.*2.0))./2.0+(m2.*(t64.^2.*2.0+t66.^2.*2.0))./2.0+(m4.*(t73.^2.*2.0+t74.^2.*2.0))./2.0+(m5.*(t89.^2.*2.0+t90.^2.*2.0))./2.0,t123,t118,t122,t121,t123,I2+I3+I5+Ir+t27+(m5.*(t71.^2.*2.0+t72.^2.*2.0))./2.0+(m4.*(t6.*t36.*2.0+t6.*t37.*2.0))./2.0+(m3.*(t7.*t36.*2.0+t7.*t37.*2.0))./2.0+(m2.*(t8.*t36.*2.0+t8.*t37.*2.0))./2.0,t115,t109,t110,t118,t115,I5+t27+(m5.*(t9.*t24.^2.*2.0+t9.*t26.^2.*2.0))./2.0,t59,t60,t122,t109,t59,t41,0.0,t121,t110,t60,0.0,t41],[5,5]);
