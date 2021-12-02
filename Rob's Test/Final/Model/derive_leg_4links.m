clear
name = 'leg';

%Change 1: Reorganize Symbolic variables, add new groupings
syms t real
syms th1 dth1 ddth1 real
syms th2 dth2 ddth2 real
syms th3 dth3 ddth3 th3_0 real
syms x dx ddx y dy ddy x_0 y_0 f_x f_y real
syms m1 m2 m3 m4 m5 I1 I2 I3 I4 I5 l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_E_m5 g kappa real
syms l_OA l_OB l_AC l_DE l_EF real 
syms tau1 tau2 tau3 Fx Fy real
syms Ir N real


% Group them
q   = [th1  ; th2 ; th3; x; y];      % generalized coordinates
dq  = [dth1 ; dth2 ; dth3; dx; dy];    % first time derivatives
ddq = [ddth1;ddth2; ddth3; ddx; ddy];  % second time derivatives
u   = [tau1 ; tau2; tau3; f_x; f_y];     % controls
F   = [Fx ; Fy];

p   = [m1 m2 m3 m4 m5 I1 I2 I3 I4 I5 Ir N l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_E_m5 l_OA l_OB l_AC l_DE l_EF g kappa th3_0]';        % parameters

% Generate Vectors and Derivativess
ihat = [0; -1; 0];
jhat = [1; 0; 0];

xhat = [1; 0; 0];
yhat = [0; 1; 0];

khat = cross(ihat,jhat);
e1hat =  cos(th1)*ihat + sin(th1)*jhat;
e2hat =  cos(th1+th2)*ihat + sin(th1+th2)*jhat;
e3hat =  cos(th1+th2+th3)*ihat + sin(th1+th2+th3)*jhat;

ddt = @(r) jacobian(r,[q;dq])*[dq;ddq]; % a handy anonymous function for taking time derivatives

rO = x*xhat + y*yhat;
rA = x*xhat + y*yhat + l_OA * e1hat;
rB = x*xhat + y*yhat + l_OB * e1hat;
rC = rA  + l_AC * e2hat;
rD = rB  + l_AC * e2hat;
rE = rD  + l_DE * e1hat;
rF = rE + l_EF * e3hat;

r_m1 = x*xhat + y*yhat + l_O_m1 * e1hat;
r_m2 = rB + l_B_m2 * e2hat;
r_m3 = rA + l_A_m3 * e2hat;
r_m4 = rC + l_C_m4 * e1hat;
r_m5 = rE + l_E_m5 * e3hat;

drA = ddt(rA);
drB = ddt(rB);
drC = ddt(rC);
drD = ddt(rD);
drE = ddt(rE);
drF = ddt(rF);
drO = ddt(rO);

dr_m1 = ddt(r_m1);
dr_m2 = ddt(r_m2);
dr_m3 = ddt(r_m3);
dr_m4 = ddt(r_m4);
dr_m5 = ddt(r_m5);

% Calculate Kinetic Energy, Potential Energy, and Generalized Forces
F2Q = @(F,r) simplify(jacobian(r,q)'*(F));    % force contributions to generalized forces
M2Q = @(M,w) simplify(jacobian(w,dq)'*(M));   % moment contributions to generalized forces


omega1 = dth1;
omega2 = dth1 + dth2;
omega3 = dth1 + dth2;
omega4 = dth1;
omega5 = dth1+dth2+dth3;

T1 = (1/2)*m1 * dot(dr_m1,dr_m1) + (1/2) * I1 * omega1^2;
T2 = (1/2)*m2 * dot(dr_m2,dr_m2) + (1/2) * I2 * omega2^2;
T3 = (1/2)*m3 * dot(dr_m3,dr_m3) + (1/2) * I3 * omega3^2;
T4 = (1/2)*m4 * dot(dr_m4,dr_m4) + (1/2) * I4 * omega4^2;
T5 = (1/2)*m5 * dot(dr_m5,dr_m5) + (1/2) * I5 * omega5^2;

T1r = (1/2)*Ir*(N*dth1)^2;
T2r = (1/2)*Ir*(dth1 + N*dth2)^2;
T3r = (1/2)*Ir*(dth1 + dth2 + N*dth3)^2;

Vg1 = m1*g*dot(r_m1, -ihat);
Vg2 = m2*g*dot(r_m2, -ihat);
Vg3 = m3*g*dot(r_m3, -ihat);
Vg4 = m4*g*dot(r_m4, -ihat);
Vg5 = m5*g*dot(r_m5, -ihat);
Ve1 = 1/2*kappa*(th3-th3_0)^2;

T = simplify(T1 + T2 + T3 + T4 + T5 + T1r + T2r + T3r);
Vg = Vg1 + Vg2 + Vg3 + Vg4 + Vg5 + Ve1;
Q_tau1 = M2Q(tau1*khat,omega1*khat);
Q_tau2 = M2Q(tau2*khat,omega2*khat); 
Q_tau2R= M2Q(-tau2*khat,omega1*khat);


Q_tau = Q_tau1+Q_tau2 + Q_tau2R;

Q = Q_tau;

% Assemble the array of cartesian coordinates of the key points
keypoints = [rA(1:2) rB(1:2) rC(1:2) rD(1:2) rE(1:2) rF(1:2) rO(1:2)];

%% All the work is done!  Just turn the crank...
% Derive Energy Function and Equations of Motion
E = T+Vg;
L = T-Vg;
eom = ddt(jacobian(L,dq).') - jacobian(L,q).' - Q;



% Rearrange Equations of Motion
A = jacobian(eom,ddq);
b = A*ddq - eom;

% Equations of motion are
% eom = A *ddq + (coriolis term) + (gravitational term) - Q = 0
Mass_Joint_Sp = A;
Grav_Joint_Sp = simplify(jacobian(Vg, q)');
Corr_Joint_Sp = simplify( eom + Q - Grav_Joint_Sp - A*ddq);

% Compute all Jacobians

J_a = jacobian(rA,q);
J_b = jacobian(rB,q);
J_c = jacobian(rC,q);
J_d = jacobian(rD,q);
J_e = jacobian(rE,q);
J_f = jacobian(rF,q);

% Compute ddt( J )
dJ_a= reshape( ddt(J_a(:)) , size(J_a) );
dJ_b= reshape( ddt(J_b(:)) , size(J_b) );
dJ_c= reshape( ddt(J_c(:)) , size(J_c) );
dJ_d= reshape( ddt(J_d(:)) , size(J_d) );
dJ_e= reshape( ddt(J_e(:)) , size(J_e) );
dJ_f= reshape( ddt(J_f(:)) , size(J_f) );

% Write Energy Function and Equations of Motion
z  = [q ; dq];

rA = rA(1:2);
drA= drA(1:2);
rB = rB(1:2);
drB= drB(1:2);
rC = rC(1:2);
drC= drC(1:2);
rD = rD(1:2);
drD= drD(1:2);
rE = rE(1:2); % to some degree we do want the ankle to be touching,no?
drE= drE(1:2);
rF = rF(1:2);
drF= drF(1:2);
rO = rO(1:2);
drO= drO(1:2);

J_a  = J_a(1:2,1:5);
dJ_a = dJ_a(1:2,1:5);
J_b  = J_b(1:2,1:5);
dJ_b = dJ_b(1:2,1:5);
J_c  = J_c(1:2,1:5);
dJ_c = dJ_c(1:2,1:5);
J_d  = J_d(1:2,1:5);
dJ_d = dJ_d(1:2,1:5);
J_e  = J_e(1:2,1:5);
dJ_e = dJ_e(1:2,1:5);
J_f  = J_f(1:2,1:5);
dJ_f = dJ_f(1:2,1:5);


directory = '../AutoDerived/';

%Joint Mass Matrix and other usefuls
matlabFunction(A,'file',[directory 'A_' name],'vars',{z p});
matlabFunction(b,'file',[directory 'b_' name],'vars',{z u p});
matlabFunction(E,'file',[directory 'energy_' name],'vars',{z p});

% Position and velocity
matlabFunction(rA,'file',[directory 'position_rA'],'vars',{z p});
matlabFunction(drA,'file',[directory 'velocity_rA'],'vars',{z p});
matlabFunction(rB,'file',[directory 'position_rB'],'vars',{z p});
matlabFunction(drB,'file',[directory 'velocity_rB'],'vars',{z p});
matlabFunction(rC,'file',[directory 'position_rC'],'vars',{z p});
matlabFunction(drC,'file',[directory 'velocity_rC'],'vars',{z p});
matlabFunction(rD,'file',[directory 'position_rD'],'vars',{z p});
matlabFunction(drD,'file',[directory 'velocity_rD'],'vars',{z p});
matlabFunction(rE,'file',[directory 'position_foot'],'vars',{z p});
matlabFunction(drE,'file',[directory 'velocity_foot'],'vars',{z p});
matlabFunction(rF,'file',[directory 'position_foot_rF'],'vars',{z p});
matlabFunction(drF,'file',[directory 'velocity_foot_rF'],'vars',{z p});
matlabFunction(rO,'file',[directory 'position_rO'],'vars',{z p});
matlabFunction(drO,'file',[directory 'velocity_rO'],'vars',{z p});

%Jacobians for intermediate points
matlabFunction(J_a ,'file',[directory 'jacobian_A'],'vars',{z p});
matlabFunction(dJ_a ,'file',[directory 'jacobian_dot_A'],'vars',{z p});
matlabFunction(J_b ,'file',[directory 'jacobian_B'],'vars',{z p});
matlabFunction(dJ_b ,'file',[directory 'jacobian_dot_B'],'vars',{z p});
matlabFunction(J_c ,'file',[directory 'jacobian_C'],'vars',{z p});
matlabFunction(dJ_c ,'file',[directory 'jacobian_dot_C'],'vars',{z p});
matlabFunction(J_d ,'file',[directory 'jacobian_D'],'vars',{z p});
matlabFunction(dJ_d ,'file',[directory 'jacobian_dot_D'],'vars',{z p});
matlabFunction(J_e ,'file',[directory 'jacobian_E'],'vars',{z p});
matlabFunction(dJ_e ,'file',[directory 'jacobian_dot_E'],'vars',{z p});
matlabFunction(J_f ,'file',[directory 'jacobian_foot'],'vars',{z p});
matlabFunction(dJ_f ,'file',[directory 'jacobian_dot_foot'],'vars',{z p});

% Joint Coriolis, Joint Gravitational
matlabFunction(Grav_Joint_Sp ,'file', [directory 'Grav_leg'] ,'vars',{z p});
matlabFunction(Corr_Joint_Sp ,'file', [directory 'Corr_leg']     ,'vars',{z p});
matlabFunction(keypoints,'file',[directory 'keypoints_' name],'vars',{z p});

