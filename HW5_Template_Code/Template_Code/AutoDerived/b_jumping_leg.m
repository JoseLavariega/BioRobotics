function b = b_jumping_leg(in1,tau,Fy,in4)
%B_JUMPING_LEG
%    B = B_JUMPING_LEG(IN1,TAU,FY,IN4)

%    This function was generated by the Symbolic Math Toolbox version 6.1.
%    16-Oct-2015 09:57:42

c1 = in4(2,:);
c2 = in4(3,:);
dth = in1(4,:);
dy = in1(3,:);
g = in4(9,:);
l = in4(1,:);
m1 = in4(4,:);
m2 = in4(5,:);
mh = in4(6,:);
th = in1(2,:);
t2 = sin(th);
t3 = cos(th);
t4 = c2.*t2;
t5 = l.*t2;
t6 = c2.*t3;
t7 = t4+t5;
t8 = l.*t3;
t9 = t6+t8;
t10 = dth.*t9;
t11 = dy+t10;
t12 = t6-t8;
t13 = t4-t5;
t14 = c1.^2;
t15 = dth.^2;
t16 = c1.*dth.*t3;
t17 = dy+t16;
t18 = dth.*l.*t3.*2.0;
t19 = dy+t18;
b = [Fy-g.*m1-g.*m2-g.*mh+dth.*(dth.*m2.*t7+c1.*dth.*m1.*t2+dth.*l.*mh.*t2.*2.0);tau-m1.*(c1.*dth.*t2.*t17.*2.0-t2.*t3.*t14.*t15.*2.0).*(1.0./2.0)-m2.*(dth.*t7.*t11.*2.0-t12.*t13.*t15.*2.0).*(1.0./2.0)+dth.*(m2.*(t7.*t11.*2.0+dth.*t7.*t9.*2.0-dth.*t12.*t13.*4.0).*(1.0./2.0)+m1.*(c1.*t2.*t17.*2.0-dth.*t2.*t3.*t14.*2.0).*(1.0./2.0)+l.*mh.*t2.*t19.*2.0+dth.*l.^2.*mh.*t2.*t3.*4.0)-g.*m2.*t9-c1.*g.*m1.*t3-g.*l.*mh.*t3.*2.0-dth.*l.*mh.*t2.*t19.*2.0];
