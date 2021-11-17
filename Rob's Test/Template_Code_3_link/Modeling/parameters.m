function p = parameters() 
 l = .25;
 m1  = .4*l;
 m2 = .4*l;
 m3 = 0.4*l;
 mh = .1;
 I1 = m1*l^2/12;
 I2 = m2*l^2/12;
 I3 = m3*l^2/12;
 c1 = l/2;
 c2 = l/2;
 c3 = l/2;
 g = 9.81;
 p = [l; c1; c2; c3; m1; m2; m3; mh; I1; I2; I3; g];
end