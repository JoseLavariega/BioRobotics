function A = A_jumping_leg(in1,in2)
%A_JUMPING_LEG
%    A = A_JUMPING_LEG(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    16-Nov-2021 23:16:08

I1 = in2(7,:);
I2 = in2(8,:);
c1 = in2(2,:);
c2 = in2(3,:);
l = in2(1,:);
m1 = in2(4,:);
m2 = in2(5,:);
mh = in2(6,:);
th1 = in1(2,:);
t2 = cos(th1);
t3 = c2.*t2.*2.0;
t4 = l.*t2.*2.0;
t5 = t3+t4;
t6 = (m2.*t5)./2.0;
t7 = c1.*m1.*t2;
t8 = l.*mh.*t2.*2.0;
t9 = t6+t7+t8;
t10 = sin(th1);
t11 = c2.*t10-l.*t10;
t12 = c2.*t2+l.*t2;
t13 = c1.^2;
t14 = t2.^2;
A = reshape([m1+m2+mh,t9,t9,I1+I2+(m1.*(t13.*t14.*2.0+t10.^2.*t13.*2.0))./2.0+(m2.*(t11.^2.*2.0+t12.^2.*2.0))./2.0+l.^2.*mh.*t14.*4.0],[2,2]);