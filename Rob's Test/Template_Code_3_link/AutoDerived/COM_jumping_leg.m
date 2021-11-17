function COM = COM_jumping_leg(in1,in2)
%COM_JUMPING_LEG
%    COM = COM_JUMPING_LEG(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    17-Nov-2021 02:25:57

c1 = in2(2,:);
c2 = in2(3,:);
c3 = in2(4,:);
dth1 = in1(5,:);
dth2 = in1(6,:);
dy = in1(4,:);
l = in2(1,:);
m1 = in2(5,:);
m2 = in2(6,:);
m3 = in2(7,:);
mh = in2(8,:);
th1 = in1(2,:);
th2 = in1(3,:);
y = in1(1,:);
t2 = cos(th1);
t3 = th1+th2;
t4 = cos(t3);
t5 = m1+m2+m3+mh;
t6 = 1.0./t5;
t7 = sin(th1);
t8 = sin(t3);
t9 = l.*t7.*2.0;
t10 = c2.*t7;
t11 = l.*t7;
t12 = c3.*m3.*t8;
t13 = l.*mh.*t8;
t14 = c3.*m3.*t4;
t15 = l.*mh.*t4;
t16 = l.*t2.*2.0;
t17 = c2.*t2;
t18 = c1.*m1.*t2;
COM = [-t6.*(t14+t15+t18-m2.*(t17-l.*t2));t6.*(m3.*(t9+y+c3.*t8)+mh.*(t9+y+l.*t8)+m2.*(t10+t11+y)+m1.*(y+c1.*t7));dth2.*t6.*(t12+t13)+dth1.*t6.*(t12+t13-m2.*(t10-t11)+c1.*m1.*t7);dy+dth2.*t6.*(t14+t15)+dth1.*t6.*(t18+m3.*(t16+c3.*t4)+m2.*(t17+l.*t2)+mh.*(t16+l.*t4))];
