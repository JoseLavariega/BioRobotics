function leg_derive_everything()
clear
name = 'jumping_leg';

%Variables for time and generalized coordinates
syms t real
syms th1 dth1 ddth1 real
syms th2 dth2 ddth2 real
syms th3 dth3 ddth3 th3_0 real
syms x dx ddx y dy ddy real
syms m0 m1 m2 m3 m4 m5 I1 I2 I3 I4 I5 l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_E_m5 g kappa real
syms l_OA l_OB l_AC l_DE l_EF real 
syms tau1 tau2 tau3 Fx Fy real
syms Ir N real

%Group them
q   = [th1  ; th2 ; th3; x; y];      % generalized coordinates
dq  = [dth1 ; dth2 ; dth3; dx; dy];    % first time derivatives
ddq = [ddth1;ddth2; ddth3; ddx; ddy];  % second time derivatives
u   = [tau1 ; tau2; tau3];              % controls
F   = [Fx ; Fy];                       % Forces

p   = [m0 m1 m2 m3 m4 m5 I1 I2 I3 I4 I5 Ir N... 
       l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_E_m5...
       l_OA l_OB l_AC l_DE l_EF...
       g kappa th3_0]';        % parameters
   
   % think about this some more

ddt = @(r) jacobian(r,[q;dq])*[dq;ddq]; % a handy anonymous function for taking time derivatives

ihat = [0; -1; 0];
jhat = [1; 0; 0]; %positive y is up w y = 0 @ the ground
khat = cross(ihat,jhat);
% 
% xhat = [1; 0; 0];
% yhat = [0; 1; 0];

e1hat =  cos(th1)*ihat + sin(th1)*jhat;
e2hat =  cos(th1+th2)*ihat + sin(th1+th2)*jhat;
e3hat =  cos(th1+th2+th3)*ihat + sin(th1+th2+th3)*jhat;

rO = x*jhat - y*ihat; %vector to origin (hip/motor 1)

rA = rO + l_OA * e1hat;
rB = rO + l_OB * e1hat;
rC = rA  + l_AC * e2hat;
rD = rB  + l_AC * e2hat;
rE = rD  + l_DE * e1hat;
rF = rE + l_EF * e3hat;

drO = ddt(rO); 
drA = ddt(rA);
drB = ddt(rB);
drC = ddt(rC);
drD = ddt(rD);
drE = ddt(rE);
drF = ddt(rF);

%vectors to link com
r_m0 = rO; %mass of motor 1??
r_m1 = l_O_m1 * e1hat;
r_m2 = rB + l_B_m2 * e2hat;
r_m3 = rA + l_A_m3 * e2hat;
r_m4 = rC + l_C_m4 * e1hat;
r_m5 = rE + l_E_m5 * e3hat;

dr_m0 = ddt(r_m0);
dr_m1 = ddt(r_m1);
dr_m2 = ddt(r_m2);
dr_m3 = ddt(r_m3);
dr_m4 = ddt(r_m4);
dr_m5 = ddt(r_m5);

% Calculate Kinetic Energy, Potential Energy, and Generalized Forces
F2Q = @(F,r) simplify(jacobian(r,q)'*(F));    % force contributions to generalized forces
M2Q = @(M,w) simplify(jacobian(w,dq)'*(M));   % moment contributions to generalized forces

%do we need an omega for the origin?
% omega0 = dth0;
dth0 = 0;
omega1 = dth0 + dth1;
omega2 = dth0 + dth1 + dth2;
omega3 = dth0 + dth1 + dth2;
omega4 = dth0 + dth1;
omega5 = dth0 + dth1 + dth2 + dth3;

% T0 = (1/2)*m0 * dot(dr_m0,dr_m0) + (1/2) * I0 * omega0^2; %kinetic energy of the hip/origin
T1 = (1/2)*m1 * dot(dr_m1,dr_m1) + (1/2) * I1 * omega1^2;
T2 = (1/2)*m2 * dot(dr_m2,dr_m2) + (1/2) * I2 * omega2^2;
T3 = (1/2)*m3 * dot(dr_m3,dr_m3) + (1/2) * I3 * omega3^2;
T4 = (1/2)*m4 * dot(dr_m4,dr_m4) + (1/2) * I4 * omega4^2;
T5 = (1/2)*m5 * dot(dr_m5,dr_m5) + (1/2) * I5 * omega5^2;

% Vg0 = m0*g*dot(r_m0, -ihat); %motor 1 mass
Vg1 = m1*g*dot(r_m1, -ihat);
Vg2 = m2*g*dot(r_m2, -ihat);
Vg3 = m3*g*dot(r_m3, -ihat);
Vg4 = m4*g*dot(r_m4, -ihat);
Vg5 = m5*g*dot(r_m5, -ihat);
Ve1 = 1/2*kappa*(th3-th3_0)^2; %elestic evergy?

%total kinetic and potential energy
T = T1 + T2 + T3 + T4 + T5;
Vg0 = 0;
Vg = Vg0 + Vg1 + Vg2 + Vg3 + Vg4 + Vg5 + Ve1;

%torques
Q_tau1 = M2Q(tau1*khat,omega1*khat);
Q_tau2 = M2Q(tau2*khat, omega2*khat);
Q_tau3 = M2Q(tau3*khat, omega3*khat);

Q_tau = Q_tau1 + Q_tau2 + Q_tau3;
Q = Q_tau;

rcm = (m1*r_m1 + m2*r_m2 + m3*r_m3 + m4*r_m4 + m5*r_m5)/(m1+m2+m3+m4+m5);

C = rF(2);
dC = ddt(C);

keypoints = [rO(1:2) rA(1:2) rB(1:2) rC(1:2) rD(1:2) rE(1:2) rF(1:2)];
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

% Compute foot jacobian
J = jacobian(rF,q);

% Compute ddt( J )
dJ= reshape( ddt(J(:)) , size(J) );

% Write Energy Function and Equations of Motion
z  = [q ; dq];

rE = rE(1:2); % to some degree we do want the ankle to be touching,no?
drE= drE(1:2);
rF = rF(1:2);
drF= drF(1:2);

% J  = J(1:2,1:3);
% dJ = dJ(1:2,1:3);

directory = '../AutoDerived/';

matlabFunction(A,'file',[directory 'A_' name],'vars',{z p});
matlabFunction(b,'file',[directory 'b_' name],'vars',{z u p});
matlabFunction(E,'file',[directory 'energy_' name],'vars',{z p});
% matlabFunction(rE,'file',[directory 'position_foot'],'vars',{z p});
% matlabFunction(drE,'file',[directory 'velocity_foot'],'vars',{z p});
matlabFunction(rF,'file',[directory 'position_foot_rF'],'vars',{z p});
matlabFunction(drF,'file',[directory 'velocity_foot_rF'],'vars',{z p});
% matlabFunction(J ,'file',[directory 'jacobian_foot'],'vars',{z p});
% matlabFunction(dJ ,'file',[directory 'jacobian_dot_foot'],'vars',{z p});
matlabFunction(C,'file',[directory 'C_' name],'vars',{z u p});
matlabFunction(dC,'file',[directory 'dC_' name],'vars',{z u p});

drcm = ddt(rcm);             % Calculate center of mass velocity vector
COM = [rcm(1:2); drcm(1:2)]; % Concatenate x and y coordinates and speeds of center of mass in array
matlabFunction(COM,'file',[directory 'COM_' name],'vars',{z p});

matlabFunction(Grav_Joint_Sp ,'file', [directory 'Grav_leg'] ,'vars',{z p});
matlabFunction(Corr_Joint_Sp ,'file', [directory 'Corr_leg']     ,'vars',{z p});
matlabFunction(keypoints,'file',[directory 'keypoints_' name],'vars',{z p});

