function b = b_leg(in1,in2,in3)
%B_LEG
%    B = B_LEG(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    29-Nov-2021 13:56:40

dth1 = in1(6,:);
dth2 = in1(7,:);
dth3 = in1(8,:);
dx = in1(9,:);
dy = in1(10,:);
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
th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
th3_0 = in3(25,:);
t2 = cos(th1);
t3 = sin(th1);
t4 = th1+th2;
t5 = l_C_m4.*t2;
t6 = l_DE.*t2;
t7 = l_OA.*t2;
t8 = l_OB.*t2;
t9 = cos(t4);
t10 = l_C_m4.*t3;
t11 = l_DE.*t3;
t12 = l_OA.*t3;
t13 = l_OB.*t3;
t14 = sin(t4);
t15 = t4+th3;
t16 = dth1.*l_O_m1.*t2;
t17 = dth1.*l_O_m1.*t3;
t18 = cos(t15);
t19 = sin(t15);
t20 = l_AC.*t9;
t21 = l_A_m3.*t9;
t22 = l_B_m2.*t9;
t23 = l_AC.*t14;
t24 = l_A_m3.*t14;
t25 = l_B_m2.*t14;
t26 = dx+t16;
t33 = dy+t17;
t27 = dth1.*t20;
t28 = dth2.*t20;
t29 = dth1.*t21;
t30 = dth2.*t21;
t31 = dth1.*t22;
t32 = dth2.*t22;
t34 = l_E_m5.*t18;
t35 = dth1.*t23;
t36 = dth2.*t23;
t37 = dth1.*t24;
t38 = dth2.*t24;
t39 = dth1.*t25;
t40 = dth2.*t25;
t41 = l_E_m5.*t19;
t56 = t7+t21;
t57 = t8+t22;
t58 = t12+t24;
t59 = t13+t25;
t69 = t5+t7+t20;
t73 = t10+t12+t23;
t42 = dth1.*t34;
t43 = dth2.*t34;
t44 = dth3.*t34;
t45 = dth1.*t41;
t46 = dth2.*t41;
t47 = dth3.*t41;
t48 = t28.*2.0;
t49 = t30.*2.0;
t50 = t32.*2.0;
t51 = t36.*2.0;
t52 = t38.*2.0;
t53 = t40.*2.0;
t60 = dth1.*t56;
t61 = dth1.*t57;
t62 = dth1.*t58;
t63 = dth1.*t59;
t64 = t20+t34;
t65 = t23+t41;
t66 = t27+t28;
t67 = t29+t30;
t68 = t31+t32;
t70 = t35+t36;
t71 = t37+t38;
t72 = t39+t40;
t78 = dth1.*t73;
t81 = dth1.*t69;
t54 = t44.*2.0;
t55 = t47.*2.0;
t74 = dth1.*t64;
t75 = dth2.*t64;
t76 = dth1.*t65;
t77 = dth2.*t65;
t82 = t30+t60;
t83 = t32+t61;
t84 = t38+t62;
t85 = t40+t63;
t86 = t6+t8+t64;
t89 = t11+t13+t65;
t94 = t28+t81;
t95 = t42+t43+t44;
t96 = t36+t78;
t97 = t45+t46+t47;
t79 = t75.*2.0;
t80 = t77.*2.0;
t87 = dy+t84;
t88 = dy+t85;
t90 = dx+t82;
t91 = dx+t83;
t92 = dth1.*t89;
t93 = dth1.*t86;
t98 = dx+t94;
t99 = dy+t96;
t109 = t47+t76+t77;
t110 = t44+t74+t75;
t100 = t24.*t90.*2.0;
t101 = t25.*t91.*2.0;
t102 = t21.*t87.*2.0;
t103 = t22.*t88.*2.0;
t106 = t23.*t98.*2.0;
t107 = t20.*t99.*2.0;
t111 = t44+t75+t93;
t112 = t47+t77+t92;
t104 = -t100;
t105 = -t101;
t108 = -t107;
t113 = dx+t111;
t114 = dy+t112;
t115 = t41.*t113.*2.0;
t116 = t34.*t114.*2.0;
t119 = t65.*t113.*2.0;
t120 = t64.*t114.*2.0;
t117 = -t115;
t118 = -t116;
t121 = -t119;
b = [tau1-(m1.*(t17.*t26.*2.0-t16.*t33.*2.0))./2.0+(m3.*(t82.*t87.*2.0-t84.*t90.*2.0))./2.0+(m2.*(t83.*t88.*2.0-t85.*t91.*2.0))./2.0+(m4.*(t94.*t99.*2.0-t96.*t98.*2.0))./2.0+(m5.*(t111.*t114.*2.0-t112.*t113.*2.0))./2.0+dth2.*((m3.*(t100-t102-t58.*t67.*2.0+t56.*t71.*2.0))./2.0+(m2.*(t101-t103-t59.*t68.*2.0+t57.*t72.*2.0))./2.0+(m5.*(t119-t120+t86.*t109.*2.0-t89.*t110.*2.0))./2.0+(m4.*(t106+t108-t66.*t73.*2.0+t69.*t70.*2.0))./2.0)+dth1.*((m3.*(t56.*t84.*2.0-t58.*t82.*2.0-t56.*t87.*2.0+t58.*t90.*2.0))./2.0+(m2.*(t57.*t85.*2.0-t59.*t83.*2.0-t57.*t88.*2.0+t59.*t91.*2.0))./2.0+(m4.*(t69.*t96.*2.0-t73.*t94.*2.0-t69.*t99.*2.0+t73.*t98.*2.0))./2.0+(m5.*(t86.*t112.*2.0-t86.*t114.*2.0-t89.*t111.*2.0+t89.*t113.*2.0))./2.0+(m1.*(l_O_m1.*t3.*t26.*2.0-l_O_m1.*t2.*t33.*2.0))./2.0)+(dth3.*m5.*(t115+t118+t86.*t97.*2.0-t89.*t95.*2.0))./2.0-g.*m2.*t59-g.*m3.*t58-g.*m4.*t73-g.*m5.*t89-g.*l_O_m1.*m1.*t3;tau2+(m3.*(t67.*t87.*2.0-t71.*t90.*2.0))./2.0+(m2.*(t68.*t88.*2.0-t72.*t91.*2.0))./2.0+(m4.*(t66.*t99.*2.0-t70.*t98.*2.0))./2.0-(m5.*(t109.*t113.*2.0-t110.*t114.*2.0))./2.0+dth2.*((m3.*(t100-t102-t24.*t67.*2.0+t21.*t71.*2.0))./2.0+(m2.*(t101-t103-t25.*t68.*2.0+t22.*t72.*2.0))./2.0+(m5.*(t119-t120+t64.*t109.*2.0-t65.*t110.*2.0))./2.0+(m4.*(t106+t108-t23.*t66.*2.0+t20.*t70.*2.0))./2.0)+dth1.*((m3.*(t100-t102+t21.*t84.*2.0-t24.*t82.*2.0))./2.0+(m2.*(t101-t103+t22.*t85.*2.0-t25.*t83.*2.0))./2.0+(m5.*(t119-t120+t64.*t112.*2.0-t65.*t111.*2.0))./2.0+(m4.*(t106+t108+t20.*t96.*2.0-t23.*t94.*2.0))./2.0)+(dth3.*m5.*(t115+t118-t65.*t95.*2.0+t64.*t97.*2.0))./2.0-g.*m2.*t25-g.*m3.*t24-g.*m4.*t23-g.*m5.*t65;(m5.*(t95.*t114.*2.0-t97.*t113.*2.0))./2.0-(kappa.*(th3.*2.0-th3_0.*2.0))./2.0+(dth3.*m5.*(t115+t118+t34.*t97.*2.0-t41.*t95.*2.0))./2.0+(dth2.*m5.*(t115+t118+t34.*t109.*2.0-t41.*t110.*2.0))./2.0+(dth1.*m5.*(t115+t118+t34.*t112.*2.0-t41.*t111.*2.0))./2.0-g.*m5.*t41;dth2.*((m4.*(t35.*2.0+t51))./2.0+(m3.*(t37.*2.0+t52))./2.0+(m2.*(t39.*2.0+t53))./2.0+(m5.*(t55+t76.*2.0+t80))./2.0)+dth1.*(m1.*t17+(m3.*(t52+t62.*2.0))./2.0+(m2.*(t53+t63.*2.0))./2.0+(m4.*(t51+t78.*2.0))./2.0+(m5.*(t55+t80+t92.*2.0))./2.0)+(dth3.*m5.*(t45.*2.0+t46.*2.0+t55))./2.0;-g.*m1-g.*m2-g.*m3-g.*m4-g.*m5-dth2.*((m4.*(t27.*2.0+t48))./2.0+(m3.*(t29.*2.0+t49))./2.0+(m2.*(t31.*2.0+t50))./2.0+(m5.*(t54+t74.*2.0+t79))./2.0)-dth1.*(m1.*t16+(m3.*(t49+t60.*2.0))./2.0+(m2.*(t50+t61.*2.0))./2.0+(m4.*(t48+t81.*2.0))./2.0+(m5.*(t54+t79+t93.*2.0))./2.0)-(dth3.*m5.*(t42.*2.0+t43.*2.0+t54))./2.0];
