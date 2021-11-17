function keypoints = keypoints_leg(in1,in2)
%KEYPOINTS_LEG
%    KEYPOINTS = KEYPOINTS_LEG(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    10-Nov-2021 14:56:16

l_AC = in2(20,:);
l_DE = in2(21,:);
l_EF = in2(22,:);
l_OA = in2(18,:);
l_OB = in2(19,:);
th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
t2 = cos(th1);
t3 = sin(th1);
t4 = th1+th2;
t5 = l_DE.*t2;
t6 = l_OA.*t2;
t7 = l_OB.*t2;
t8 = cos(t4);
t9 = l_DE.*t3;
t10 = l_OA.*t3;
t11 = l_OB.*t3;
t12 = sin(t4);
t13 = t4+th3;
t14 = l_AC.*t8;
t15 = l_AC.*t12;
t16 = -t5;
t17 = -t6;
t18 = -t7;
t19 = -t14;
keypoints = reshape([t10,t17,t11,t18,t10+t15,t17+t19,t11+t15,t18+t19,t9+t11+t15,t16+t18+t19,t9+t11+t15+l_EF.*sin(t13),t16+t18+t19-l_EF.*cos(t13)],[2,6]);